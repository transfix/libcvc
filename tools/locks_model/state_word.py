"""The group state word of the continuous pool (S2a; S2.md S2-Q2, S2-Q4, S2-Q5, S2-Q15; td 8.3 PL-1).

An exhaustive model of `detail::group_pool_link::state` and the pool's three facts beside it
(`queued`, `host_held`, `deferred`, under the ready-queue mutex Q), as S2a.0 froze them in its
comment on `group_pool_link` (inc/cvc/state/state_exec/exec_group.h). Like revision 5b's
`gdestroy` it has no C++ port. Standard library only.

Every atomic operation is one step: each fetch_or, fetch_and, CAS, load and store of the word;
taking and releasing Q, the notifier lock NL, an ingress mutex IM and the lock port's mutex PM;
the token's take and release. What only the holder of Q reads or writes (the ready queue, the
pool's record of the group, the timer heap) is folded into the step that holds Q, and so is a
load of DETACHING under Q (only Q's holder sets it, and the word goes to 0 only after the
notifier is cleared, which a waker holding NL excludes). Every interleaving of the steps is
explored, breadth first, so each violation's trace is a shortest one. The workers are
interchangeable, so a state is kept with its workers sorted (one state per orbit of their
permutations); with one group at most one worker is past its pop at a time (pop is under Q and
the group is queued once), so a third worker adds no state.

The actors (S2-Q15):
- posters: an ingress post (IM, the flag, then wakeup::notify under NL calling wake(g)) and a lock
  post (the same under the port's PM, dropped once the port is closed);
- workers: pop (pop and PUMPING are one step under Q), clear NOTIFIED, take the token, drain and
  run (nondeterministic: what it leaves runnable, a sleeper's deadline, future or already due, a
  failure, a waiting host body), release the token, then the release under Q (the CAS);
- a timer: a filed deadline firing under Q, as wake(g) (at any time: a fire may be stale);
- hosts: with_group on another thread (wait for PUMPING, set host_held, take the token, run,
  release, re-queue if deferred), whose run may do nothing, submit directly under the token (as
  exec_group::execute does: no post, no wake) or post. One may race a detach anywhere; none races
  a teardown (a host program never uses a group it is destroying: a with_group that set
  host_held before the teardown began is the teardown's to wait for, one that had not is dropped);
- the attaching thread: attach's wake, its non-waiting host bodies (as a host), the servicing of
  waiting bodies (run under the blocked worker's token), take_failed, and last the teardown or the
  detach (S2-Q4 steps 1-5).

Properties (PL-1), each a violation name:
- every state: `double-queue` (queued twice), `queued-while-pumped`, `queued-unscheduled` (in the
  ready queue without SCHEDULED), `two-pumpers`, `pump-host-held` (a worker holds the token while
  host_held is set), `token-contended` (a take finds another thread's token: "the token is held by
  at most one thread"), `body-without-token` (a waiting body run without its worker's token);
- after the detach: `wake-after-detach` (a wake touches the word or the pool after the notifier is
  cleared), `read-after-detach` (a worker or the timer touches the group after it left the pool's
  structures), `record-after-detach` (anyone touches the pool's record of the group after the
  attachment is dropped), `use-after-free` (anything touches a destroyed group);
- at idle points (every actor between operations: posters between posts, workers and the timer
  at the top of their loops, hosts not started or done, the attaching thread between operations),
  for an attached group not detaching: `lost-wake` (a post not drained, or runnable work left,
  and the group not queued), `lost-timer` (a sleeper neither queued nor filed), `stuck-scheduled`
  (SCHEDULED set and not queued: every later wake is a no-op);
- `deadlock`: no step enabled while an actor is not at an idle point.

`--amend` adds the amendments this model proposes to the frozen transitions (README.md):
- `host-wake`: a host's release wakes the group when no pop was deferred (wake's transition under
  Q: fetch_or(S|N), queue unless SCHEDULED or DETACHING was set), since a host's run may submit or
  kill directly under the token without a post;
- `host-excl`: a host waits while PUMPING is set or another thread holds host_held (host_held
  names its holder);
- `host-detaching`: a host that finds DETACHING set, or the group no longer attached, under Q sets
  no host_held; it takes the token as for an unattached group once the detach is over (the
  attaching thread at once);
- `nested-release`: a host's release under Q reads the attachment first and leaves the pool's
  record alone when its own fn detached the group (with `--opts nested=1`).

`--mutation` breaks one transition, to show the check finds it: `release-store` (the release
clears SCHEDULED by a load and a plain store instead of the CAS), `pump-host-held` (pop pumps a
host-held group), `zero-before-clear` (the detach stores 0 before clearing the notifier),
`wake-no-recheck` (wake queues without re-reading DETACHING under Q), `pop-no-detaching` (pop
pumps a detaching group).

usage:
    python -I state_word.py [--actors posters=2,workers=2,hosts=1,bodies=1,teardown=1]
                            [--opts posts=1,timer=1,sleeps=1,waiting=0,failures=0,pre=0,nested=0]
                            [--amend all|none|NAME,...] [--mutation NAME] [--max-states N]
    python -I state_word.py --suite quick|deep     the CI checks (README.md); exit 0 when all hold

`--actors` counts posters (alternately ingress and lock), workers, hosts (other threads) and
bodies (the attaching thread's), and ends the attaching thread with teardown=1 (the default) or
detach=1 (or neither). The JSON printed has `states`, `transitions`, `idle_states`, `seconds`,
`complete` and `violations`, each with its detail, its length in steps and its trace.
"""

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))

import argparse
import collections
import json
import sys
import time

from lockmodel import Violation

# the word's bits (group_pool_link::scheduled, notified, pumping, detaching)
S, N, P, D = 1, 2, 4, 8
AMENDMENTS = ('host-wake', 'host-excl', 'host-detaching', 'nested-release')
MUTATIONS = ('release-store', 'pump-host-held', 'zero-before-clear', 'wake-no-recheck',
             'pop-no-detaching')


def wstr(w):
    return '|'.join(n for b, n in ((S, 'S'), (N, 'N'), (P, 'P'), (D, 'D')) if w & b) or '0'


# global slots of the state (a flat list of small ints; 0 is "none" for an owner, actors are 1..n)
(WORD, Q, NL, IM, PM, TOK, TOKD, QN, HH, DFR, HEAP, ATT, NTF, LEFT, CLEARED, CLOSED, DESTROYED,
 FLAGS, READY, SLEEPER, SLEEPS, FAILS, WAITB, FLIST, HQ, HQW) = range(26)
NG = 26
SLOTS = 3   # per actor

# the post routine (a poster's pc; a host's pc - H_POST)
R_START, R_NL, R_OR, R_Q, R_QUEUE, R_QREL, R_END = 1, 2, 3, 4, 5, 6, 7
# a worker
(W_IDLE, W_RELQ0, W_PREL, W_CLRN, W_TOK, W_RUN, W_WAITB, W_TREL, W_Q2, W_CLRP, W_FAIL, W_CAS,
 W_STORE, W_REL) = range(14)
W_PUMPING = range(W_PREL, W_CLRP + 1)   # PUMPING is set by this worker
# the timer
T_IDLE, T_OR, T_REL = 0, 1, 2
# a host op (another thread's with_group; the attaching thread's bodies use the same pcs)
(H_START, H_Q, H_QREL, H_TOK, H_RUN, H_TREL, H_Q2, H_CLR, H_WOR, H_REL, H_QRELU, H_U, H_URUN,
 H_UREL) = range(14)
H_POST = 20   # + R_*
H_DONE = 30
# the attaching thread
A_IDLE, A_ATT_OR, A_ATT_Q, A_ATT_QUEUE, A_ATT_REL, A_DONE = 0, 40, 41, 42, 43, 49
# the detach (S2-Q4's steps; teardown, detach, take_failed and a detach inside a body)
(D_0, D_1, D_2, D_3, D_4, D_5, D_6, D_7, D_8, D_9, D_10, D_11, D_12, D_13, D_U) = range(50, 65)
AUX_TEARDOWN, AUX_TF, AUX_NESTED = 1, 2, 4


class Config(object):
    def __init__(self, posters=2, workers=2, hosts=1, bodies=1, end='teardown', posts=1, timer=1,
                 sleeps=1, waiting=0, failures=0, pre=0, nested=0,
                 host_outcomes=('none', 'submit', 'post'), amend=(), mutation=()):
        self.posters = tuple('ingress' if i % 2 == 0 else 'lock' for i in range(posters))
        self.workers = workers
        self.hosts = hosts
        self.bodies = bodies
        self.end = end            # 'teardown', 'detach' or None
        self.posts = posts        # posts per poster
        self.timer = timer
        self.sleeps = sleeps      # sleepers a run may arm, in all
        self.waiting = waiting    # waiting host bodies, in all
        self.failures = failures  # failed pumps, in all
        self.pre = pre            # posts before attach (no wake: attach's wake must see them)
        self.nested = nested      # an attaching thread's body may detach inside its fn
        self.host_outcomes = tuple(host_outcomes)
        self.amend = frozenset(AMENDMENTS if amend == 'all' else amend)
        self.mutation = frozenset(mutation)
        for a in self.amend:
            assert a in AMENDMENTS, a
        for m in self.mutation:
            assert m in MUTATIONS, m
        assert end in ('teardown', 'detach', None)
        assert not nested or end == 'detach', 'nested=1 needs the detach end (teardown=0, detach=1)'

    def describe(self):
        return {'posters': list(self.posters), 'workers': self.workers, 'hosts': self.hosts,
                'bodies': self.bodies, 'end': self.end, 'posts': self.posts, 'timer': self.timer,
                'sleeps': self.sleeps, 'waiting': self.waiting, 'failures': self.failures,
                'pre': self.pre, 'nested': self.nested, 'host_outcomes': list(self.host_outcomes),
                'amend': sorted(self.amend), 'mutation': sorted(self.mutation)}


class Model(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.amend = cfg.amend
        self.mut = cfg.mutation
        self.names = {}
        self.kind = {}
        self.pkind = {}
        a = 1
        self.posters = []
        for i, k in enumerate(cfg.posters):
            self.posters.append(a)
            self.names[a] = 'P%d' % (i + 1)
            self.kind[a] = 'poster'
            self.pkind[a] = k
            a += 1
        self.workers = []
        for i in range(cfg.workers):
            self.workers.append(a)
            self.names[a] = 'W%d' % (i + 1)
            self.kind[a] = 'worker'
            a += 1
        self.timer = 0
        if cfg.timer:
            self.timer = a
            self.names[a] = 'T'
            self.kind[a] = 'timer'
            a += 1
        self.hosts = []
        for i in range(cfg.hosts):
            self.hosts.append(a)
            self.names[a] = 'H%d' % (i + 1)
            self.kind[a] = 'host'
            a += 1
        self.A = a
        self.names[a] = 'A'
        self.kind[a] = 'attacher'
        self.actors = list(range(1, a + 1))
        self.size = NG + SLOTS * a
        # the workers are interchangeable: a state is kept with its workers sorted (canon), the
        # owners that can name a worker (Q, the token, HQW) renamed
        self.wbase = self.base(self.workers[0]) if self.workers else 0
        self.ops = ['attach'] + ['body'] * cfg.bodies + ([cfg.end] if cfg.end else [])
        # the order of step 4's last three actions (the mutation stores 0 first)
        if 'zero-before-clear' in self.mut:
            self.tail = ('zero', 'clear', 'drop')
        else:
            self.tail = ('clear', 'drop', 'zero')

    @staticmethod
    def base(a):
        return NG + SLOTS * (a - 1)

    def canon(self, s):
        """(key, perm): `s` with its workers sorted by their slots, and the permutation giving it
        (perm[i]: the worker of `s` that is worker i of the key; None for the identity). Two
        workers with equal slots are interchangeable: no owner can name either (Q, the token and
        HQW each belong to a pc that one worker at most is at), so the sorted form is unique."""
        nw = len(self.workers)
        if nw < 2:
            return bytes(s), None
        b0 = self.wbase
        ws = [s[b0 + SLOTS * i:b0 + SLOTS * (i + 1)] for i in range(nw)]
        if all(ws[i] <= ws[i + 1] for i in range(nw - 1)):
            return bytes(s), None
        perm = sorted(range(nw), key=ws.__getitem__)
        t = list(s)
        for i, j in enumerate(perm):
            t[b0 + SLOTS * i:b0 + SLOTS * (i + 1)] = ws[j]
        w0 = self.workers[0]
        for f in (Q, TOK, HQW):
            v = s[f] - w0
            if 0 <= v < nw:
                t[f] = w0 + perm.index(v)
        return bytes(t), perm

    def real_actor(self, a, perm):
        """The actor of the real state that is actor `a` of its canonical form."""
        if perm is None or a not in self.workers:
            return a
        w0 = self.workers[0]
        return w0 + perm[a - w0]

    def initial(self):
        s = [0] * self.size
        s[ATT] = 1
        s[NTF] = 1
        s[FLAGS] = self.cfg.pre
        s[SLEEPS] = self.cfg.sleeps
        s[FAILS] = self.cfg.failures
        s[WAITB] = self.cfg.waiting
        for p in self.posters:
            s[self.base(p) + 1] = self.cfg.posts
        b = self.base(self.A)
        s[b + 1] = A_ATT_OR
        return s

    # ------------------------------------------------------------------------------------------
    # checks

    def touch(self, s, a, what, waker=False):
        """`a` touches the group's `what`: 'word', 'rec' (the pool's record: queued, host_held,
        deferred, the heap entry, the failed list), 'pool' (Q, for the group) or 'group' (its
        scheduler: the drain, the run, the token)."""
        nm = self.names[a]
        if s[DESTROYED]:
            raise Violation('use-after-free', '%s touches the %s of a destroyed group' % (nm, what))
        if waker and s[CLEARED]:
            raise Violation('wake-after-detach',
                            '%s: a wake touches the %s after the notifier was cleared' % (nm, what))
        k = self.kind[a]
        if k in ('worker', 'timer') and s[LEFT]:
            raise Violation('read-after-detach',
                            '%s touches the %s after the group left the pool' % (nm, what))
        if what == 'rec' and not s[ATT]:
            raise Violation('record-after-detach',
                            "%s touches the pool's record of the group after the detach dropped it"
                            % nm)

    def queue(self, s, a):
        self.touch(s, a, 'rec')
        if s[QN]:
            raise Violation('double-queue', '%s queues the group while it is queued' % self.names[a])
        s[QN] = 1

    def take_token(self, s, a):
        self.touch(s, a, 'group')
        if s[TOK] and s[TOK] != a:
            raise Violation('token-contended', '%s takes the token while %s holds it'
                            % (self.names[a], self.names[s[TOK]]))
        s[TOK] = a
        s[TOKD] += 1

    def release_token(self, s, a):
        s[TOKD] -= 1
        if s[TOKD] == 0:
            s[TOK] = 0

    def invariants(self, s):
        w = s[WORD]
        if s[QN] and w & P:
            raise Violation('queued-while-pumped', 'queued with PUMPING set')
        if s[QN] and s[ATT] and not w & S:
            raise Violation('queued-unscheduled', 'queued without SCHEDULED (word %s)' % wstr(w))
        pumpers = [x for x in self.workers if s[self.base(x)] in W_PUMPING]
        if len(pumpers) > 1:
            raise Violation('two-pumpers', '%s pump at once'
                            % ' and '.join(self.names[x] for x in pumpers))
        if s[TOK] and self.kind[s[TOK]] == 'worker' and s[HH]:
            raise Violation('pump-host-held', '%s holds the token while %s holds host_held'
                            % (self.names[s[TOK]], self.names[s[HH]]))

    def idle(self, s):
        for a in self.posters:
            if s[self.base(a)] != 0:
                return False
        for a in self.workers:
            if s[self.base(a)] != W_IDLE:
                return False
        if self.timer and s[self.base(self.timer)] != T_IDLE:
            return False
        for a in self.hosts:
            if s[self.base(a)] not in (H_START, H_DONE):
                return False
        return s[self.base(self.A) + 1] in (A_IDLE, A_DONE)

    def idle_check(self, s):
        if not s[ATT] or s[WORD] & D:
            return
        if (s[FLAGS] or s[READY]) and not s[QN]:
            raise Violation('lost-wake', '%s and the group is not queued (word %s)'
                            % ('a post not drained' if s[FLAGS] else 'runnable work left',
                               wstr(s[WORD])))
        if s[SLEEPER] and not (s[QN] or s[HEAP]):
            raise Violation('lost-timer', 'a sleeper neither queued nor filed in the heap')
        if s[WORD] & S and not s[QN]:
            raise Violation('stuck-scheduled', 'SCHEDULED set, the group in no queue (word %s)'
                            % wstr(s[WORD]))

    # ------------------------------------------------------------------------------------------
    # the post routine (wakeup::notify -> the notifier -> wake(g))

    def post_ok(self, s, kind, sub):
        if sub == R_START:
            if kind == 'lock':
                return not s[PM]
            return not s[IM] and not s[DESTROYED]
        if sub == R_NL:
            return not s[NL]
        if sub == R_Q:
            return not s[Q]
        return True

    def post_step(self, s, a, kind, sub, note):
        if sub == R_START:
            if kind == 'lock':
                s[PM] = a
                if s[CLOSED]:
                    note("takes PM: the port is closed, the post is dropped")
                    return R_END
                note('takes PM, appends a lock post')
            else:
                self.touch(s, a, 'group')
                s[IM] = a
                note('takes IM, pushes an ingress entry')
            s[FLAGS] += 1
            return R_NL
        if sub == R_NL:
            s[NL] = a
            if not s[NTF]:
                note('takes NL: no notifier, signals the event')
                return R_END
            note('takes NL: calls the notifier, wake(g)')
            return R_OR
        if sub == R_OR:
            self.touch(s, a, 'word', waker=True)
            old = s[WORD]
            s[WORD] |= S | N
            if old & (S | D):
                note('wake: fetch_or(S|N) finds %s: done' % wstr(old))
                return R_END
            note('wake: fetch_or(S|N) finds %s' % wstr(old))
            return R_Q
        if sub == R_Q:
            self.touch(s, a, 'pool', waker=True)
            s[Q] = a
            note('wake: takes Q')
            return R_QUEUE
        if sub == R_QUEUE:
            if s[WORD] & D and 'wake-no-recheck' not in self.mut:
                note('wake: DETACHING is set now: does not queue')
            else:
                self.queue(s, a)
                note('wake: queues g')
            return R_QREL
        if sub == R_QREL:
            s[Q] = 0
            note('wake: releases Q (wakes a parked worker)')
            return R_END
        # R_END
        if s[NL] == a:
            s[NL] = 0
        if kind == 'lock':
            s[PM] = 0
            note('releases NL, PM')
        else:
            s[IM] = 0
            note('releases NL, IM')
        return 0

    # ------------------------------------------------------------------------------------------
    # the pump's run

    def run_outcomes(self, s, work, special=True):
        outs = []
        if special:
            if s[FAILS]:
                outs.append(('fail',))
            if s[WAITB] and work:
                outs.append(('block',))
        if s[SLEEPER]:
            sl_opts = ((1, 0), (0, 0))   # (a sleeper after the run, armed by it)
        else:
            sl_opts = ((0, 0), (1, 1)) if work and s[SLEEPS] else ((0, 0),)
        for sl, armed in sl_opts:
            w = work or (s[SLEEPER] and not sl)
            for r in ((0, 1) if w else (0,)):
                for dl in ((1, 2) if sl else (0,)):
                    outs.append(('ok', r, sl, dl, armed))
        return outs

    def apply_run(self, s, a, o, note):
        b = self.base(a)
        if o[0] == 'fail':
            s[READY] = 0
            s[SLEEPER] = 0
            s[FAILS] -= 1
            s[b + 1] = 8
            s[b] = W_TREL
            note('run: a C++ exception escapes; kills the processes ("internal error")')
            return
        if o[0] == 'block':
            s[HQ] = 1
            s[HQW] = a
            s[WAITB] -= 1
            s[READY] = 1
            s[b] = W_WAITB
            note('run: a waiting host body (run_on_host_waiting): blocks, holding the token')
            return
        _, r, sl, dl, armed = o
        s[READY] = r
        s[SLEEPER] = sl
        if armed:
            s[SLEEPS] -= 1
        s[b + 1] = r | (dl << 1)
        s[b] = W_TREL
        note('run: ends with runnable=%s, next_deadline=%s%s'
             % (bool(r), ('none', 'future', 'due')[dl], ' (armed a sleeper)' if armed else ''))

    # ------------------------------------------------------------------------------------------
    # the enabled steps of each actor: lists of labels

    def options(self, s, a):
        k = self.kind[a]
        b = self.base(a)
        pc = s[b]
        if k == 'poster':
            if pc == 0:
                return ['post'] if s[b + 1] and self.post_ok(s, self.pkind[a], R_START) else []
            return ['step'] if self.post_ok(s, self.pkind[a], pc) else []
        if k == 'worker':
            if pc == W_IDLE:
                return ['pop'] if s[QN] and not s[Q] else []
            if pc == W_Q2:
                return ['step'] if not s[Q] else []
            if pc == W_RUN:
                if s[IM] or s[PM]:
                    return []
                return self.run_outcomes(s, bool(s[READY] or s[FLAGS]))
            if pc == W_WAITB:
                return self.run_outcomes(s, True, False) if s[HQ] == 2 else []
            return ['step']
        if k == 'timer':
            if pc == T_IDLE:
                return ['fire'] if s[HEAP] and not s[Q] else []
            return ['step']
        if k == 'host':
            return self.host_options(s, a, pc)
        return self.attacher_options(s, a, b)

    def teardown_begun(self, s):
        """Whether the attaching thread has begun the group's teardown (its last op)."""
        if self.cfg.end != 'teardown':
            return False
        b = self.base(self.A)
        last = len(self.ops) - 1
        return s[b] > last or (s[b] == last and s[b + 1] != A_IDLE and s[b + 2] & AUX_TEARDOWN)

    def host_options(self, s, a, pc):
        # Another thread's with_group may race a detach (its entry, its host_held, its release),
        # but not a teardown: a host program never uses a group it is destroying, so there only a
        # with_group that set host_held before DETACHING is the teardown's to wait for (step 2),
        # and one that had not is abandoned.
        racing_teardown = a != self.A and self.teardown_begun(s)
        if pc == H_START:
            if s[DESTROYED] or racing_teardown:
                return []
            return ['start']
        if pc == H_DONE:
            return []
        if pc == H_Q:
            if racing_teardown:
                return ['abandon']
            if s[Q]:
                return []
            if 'host-detaching' in self.amend and (s[WORD] & D or not s[ATT]):
                return ['step']   # it will not wait: it takes the unattached path
            if s[WORD] & P:
                return []
            if 'host-excl' in self.amend and s[HH] and s[HH] != a:
                return []
            return ['step']
        if pc == H_Q2:
            return ['step'] if not s[Q] else []
        if pc == H_RUN:
            outs = list(self.cfg.host_outcomes)
            if a == self.A and self.cfg.nested:
                outs.append('detach')
            return outs
        if pc > H_POST and pc < H_DONE:
            return ['step'] if self.post_ok(s, 'ingress', pc - H_POST) else []
        if pc == H_U:
            if s[DESTROYED]:
                return ['end']
            if s[TOK] or (a != self.A and s[ATT]):
                return []
            return ['step']
        if pc == H_URUN:
            return ['none', 'submit']
        return ['step']

    def attacher_options(self, s, a, b):
        pc = s[b + 1]
        opi = s[b]
        if pc == A_DONE:
            return []
        if pc == A_IDLE:
            outs = []
            if opi < len(self.ops):
                outs.append('op')
            if s[HQ] == 1:
                outs.append('service')
            if s[FLIST] and s[ATT] and not s[Q]:
                outs.append('take_failed')
            return outs
        if pc in (A_ATT_Q, D_0, D_7):
            return ['step'] if not s[Q] else []
        if pc == D_3:
            outs = []
            if not s[Q] and not s[WORD] & P and s[HH] in (0, a):
                outs.append('step')
            if s[HQ] == 1:
                outs.append('service')
            return outs
        if pc == D_5:
            return ['step'] if s[TOK] in (0, a) else []
        if pc == D_6:
            return ['step'] if not s[PM] else []
        if pc == D_10 or pc == D_11 or pc == D_12:
            act = self.tail[pc - D_10]
            if act == 'clear' and s[NL]:
                return []
            return ['step']
        if pc == D_13:
            if s[b + 2] & AUX_TEARDOWN and (s[IM] or s[NL] or s[PM]):
                return []
            return ['step']
        if pc == D_U:
            return ['step'] if not s[TOK] else []
        if pc < 40:   # a body: the host pcs
            outs = self.host_options(s, a, pc)
            if pc == H_Q and s[HQ] == 1:
                outs = outs + ['service']
            return outs
        return ['step']

    # ------------------------------------------------------------------------------------------
    # steps

    def apply(self, s, a, label, log=None):
        notes = []
        note = notes.append
        k = self.kind[a]
        if k == 'poster':
            self.step_poster(s, a, note)
        elif k == 'worker':
            self.step_worker(s, a, label, note)
        elif k == 'timer':
            self.step_timer(s, a, note)
        elif k == 'host':
            self.step_host(s, a, self.base(a), label, note)
        else:
            self.step_attacher(s, a, label, note)
        if log is not None:
            log.append('; '.join(notes))

    def step_poster(self, s, a, note):
        b = self.base(a)
        pc = s[b] or R_START
        nxt = self.post_step(s, a, self.pkind[a], pc, note)
        if nxt == 0:
            s[b + 1] -= 1
        s[b] = nxt

    def step_worker(self, s, a, label, note):
        b = self.base(a)
        pc = s[b]
        if pc == W_IDLE:
            s[Q] = a
            self.touch(s, a, 'rec')
            s[QN] = 0
            self.touch(s, a, 'word')
            if s[WORD] & D and 'pop-no-detaching' not in self.mut:
                note('takes Q, pops g: DETACHING: drops it')
                s[b] = W_RELQ0
            elif s[HH] and 'pump-host-held' not in self.mut:
                s[WORD] |= N
                s[DFR] = 1
                note('takes Q, pops g: host_held: fetch_or(N), sets deferred, takes the next')
                s[b] = W_RELQ0
            else:
                s[WORD] |= P
                note('takes Q, pops g, fetch_or(P)')
                s[b] = W_PREL
        elif pc == W_RELQ0 or pc == W_PREL:
            s[Q] = 0
            note('releases Q')
            s[b] = W_IDLE if pc == W_RELQ0 else W_CLRN
        elif pc == W_CLRN:
            self.touch(s, a, 'word')
            s[WORD] &= ~N
            note('fetch_and(~N)')
            s[b] = W_TOK
        elif pc == W_TOK:
            self.take_token(s, a)
            note('takes the token')
            s[b] = W_RUN
        elif pc == W_RUN:
            self.touch(s, a, 'group')
            drained = s[FLAGS]
            s[FLAGS] = 0
            if drained:
                note('drains %d post(s)' % drained)
            self.apply_run(s, a, label, note)
        elif pc == W_WAITB:
            self.touch(s, a, 'group')
            s[HQ] = 0
            s[HQW] = 0
            note('the waiting body has run')
            self.apply_run(s, a, label, note)
        elif pc == W_TREL:
            self.touch(s, a, 'group')
            self.release_token(s, a)
            note('releases the token')
            s[b] = W_Q2
        elif pc == W_Q2:
            s[Q] = a
            note('takes Q')
            s[b] = W_CLRP
        elif pc == W_CLRP:
            self.touch(s, a, 'word')
            s[WORD] &= ~P
            res = s[b + 1]
            if s[WORD] & D:
                note('release: fetch_and(~P), DETACHING: done')
                s[b] = W_REL
            elif res & 8:
                note('release: fetch_and(~P), the pump failed')
                s[b] = W_FAIL
            elif res & 1 or (res >> 1) == 2:
                self.queue(s, a)
                note('release: fetch_and(~P); %s: queues g again'
                     % ('runnable' if res & 1 else 'next_deadline due'))
                s[b] = W_REL
            else:
                note('release: fetch_and(~P)')
                s[b] = W_CAS
        elif pc == W_FAIL:
            self.touch(s, a, 'word')
            s[WORD] |= D
            self.touch(s, a, 'rec')
            s[FLIST] = 1
            note('failure: fetch_or(D), lists g on its attaching thread\'s failed list')
            s[b] = W_REL
        elif pc == W_CAS:
            self.touch(s, a, 'word')
            w = s[WORD]
            if 'release-store' in self.mut:
                if w & N:
                    self.queue(s, a)
                    note('release (mutated): loads %s: queues g again' % wstr(w))
                    s[b] = W_REL
                else:
                    note('release (mutated): loads %s' % wstr(w))
                    s[b] = W_STORE
            elif w == S:
                s[WORD] = 0
                if (s[b + 1] >> 1) == 1:
                    self.touch(s, a, 'rec')
                    s[HEAP] = 1
                    note('release: CAS(S -> 0) succeeds; files next_deadline in the heap')
                else:
                    note('release: CAS(S -> 0) succeeds')
                s[b] = W_REL
            else:
                self.queue(s, a)
                note('release: CAS(S -> 0) fails on %s: queues g again' % wstr(w))
                s[b] = W_REL
        elif pc == W_STORE:
            self.touch(s, a, 'word')
            s[WORD] = 0
            if (s[b + 1] >> 1) == 1:
                self.touch(s, a, 'rec')
                s[HEAP] = 1
            note('release (mutated): stores 0')
            s[b] = W_REL
        elif pc == W_REL:
            s[Q] = 0
            note('releases Q')
            s[b] = W_IDLE
            s[b + 1] = 0

    def step_timer(self, s, a, note):
        b = self.base(a)
        pc = s[b]
        if pc == T_IDLE:
            s[Q] = a
            self.touch(s, a, 'rec')
            s[HEAP] = 0
            if s[WORD] & D or not s[ATT]:
                note('takes Q, the deadline falls due: g is detaching: nothing')
                s[b] = T_REL
            else:
                note('takes Q, the deadline falls due')
                s[b] = T_OR
        elif pc == T_OR:
            self.touch(s, a, 'word')
            old = s[WORD]
            s[WORD] |= S | N
            if old & (S | D):
                note('as wake: fetch_or(S|N) finds %s: done' % wstr(old))
            else:
                self.queue(s, a)
                note('as wake: fetch_or(S|N) finds %s: queues g' % wstr(old))
            s[b] = T_REL
        else:
            s[Q] = 0
            note('releases Q')
            s[b] = T_IDLE

    def host_done(self, s, a, b):
        if a == self.A:
            s[b] += 1
            s[b + 1] = A_IDLE
            s[b + 2] = 0
        else:
            s[b] = H_DONE

    def step_host(self, s, a, b, label, note):
        """A host op: another thread's with_group (pc in s[b]), or a body of the attaching thread
        (pc in s[b + 1], b the attaching thread's base)."""
        pi = b + 1 if a == self.A else b
        pc = s[pi]
        if pc == H_START:
            if s[ATT]:
                note('with_group: the group is attached')
                s[pi] = H_Q
            else:
                note('with_group: the group is not attached')
                s[pi] = H_U
        elif pc == H_Q:
            if label == 'abandon':
                note('the teardown has begun: the host program does not go on (abandoned)')
                s[pi] = H_DONE
                return
            s[Q] = a
            self.touch(s, a, 'word')
            if 'host-detaching' in self.amend and (s[WORD] & D or not s[ATT]):
                note('takes Q (PUMPING clear): %s: no host_held'
                     % ('DETACHING is set' if s[WORD] & D else 'no longer attached'))
                s[pi] = H_QRELU
            else:
                self.touch(s, a, 'rec')
                s[HH] = a
                note('takes Q (PUMPING clear), sets host_held')
                s[pi] = H_QREL
        elif pc == H_QREL or pc == H_QRELU:
            s[Q] = 0
            note('releases Q')
            s[pi] = H_TOK if pc == H_QREL else H_U
        elif pc == H_TOK:
            self.take_token(s, a)
            note('takes the token')
            s[pi] = H_RUN
        elif pc == H_RUN:
            self.touch(s, a, 'group')
            if label == 'none':
                note('runs fn: nothing for the pool')
                s[pi] = H_TREL
            elif label == 'submit':
                s[READY] = 1
                note('runs fn: submits a process directly (exec_group::execute: no post)')
                s[pi] = H_TREL
            elif label == 'post':
                note('runs fn: posts to the group')
                s[pi] = H_POST + R_START
            else:   # 'detach', the attaching thread's body only
                note('runs fn: detaches the group (pool.detach on this thread)')
                s[b + 1] = D_0
                s[b + 2] = AUX_NESTED
        elif pc > H_POST and pc < H_DONE:
            nxt = self.post_step(s, a, 'ingress', pc - H_POST, note)
            s[pi] = H_POST + nxt if nxt else H_TREL
        elif pc == H_TREL:
            self.touch(s, a, 'group')
            self.release_token(s, a)
            note('releases the token')
            s[pi] = H_Q2
        elif pc == H_Q2:
            s[Q] = a
            note('takes Q')
            s[pi] = H_CLR
        elif pc == H_CLR:
            if 'nested-release' in self.amend and not s[ATT]:
                note('the group is no longer attached (fn detached it): leaves the record alone')
                s[pi] = H_REL
                return
            self.touch(s, a, 'rec')
            s[HH] = 0
            if s[DFR]:
                s[DFR] = 0
                self.queue(s, a)
                note('clears host_held; deferred: clears it, queues g again')
                s[pi] = H_REL
            elif 'host-wake' in self.amend:
                note('clears host_held (not deferred)')
                s[pi] = H_WOR
            else:
                note('clears host_held (not deferred)')
                s[pi] = H_REL
        elif pc == H_WOR:
            self.touch(s, a, 'word')
            old = s[WORD]
            s[WORD] |= S | N
            if old & (S | D):
                note('as wake: fetch_or(S|N) finds %s: done' % wstr(old))
            else:
                self.queue(s, a)
                note('as wake: fetch_or(S|N) finds %s: queues g' % wstr(old))
            s[pi] = H_REL
        elif pc == H_REL:
            s[Q] = 0
            note('releases Q')
            self.host_done(s, a, b)
        elif pc == H_U:
            if label == 'end':
                note('the group is gone: the host does not use it')
                self.host_done(s, a, b)
                return
            self.take_token(s, a)
            note('takes the token as for an unattached group')
            s[pi] = H_URUN
        elif pc == H_URUN:
            self.touch(s, a, 'group')
            if label == 'submit':
                s[READY] = 1
                note('runs fn: submits a process (an unattached group: pump_all pumps it)')
            else:
                note('runs fn: nothing for the pool')
            s[pi] = H_UREL
        elif pc == H_UREL:
            self.release_token(s, a)
            note('releases the token')
            self.host_done(s, a, b)

    def step_attacher(self, s, a, label, note):
        b = self.base(a)
        opi = s[b]
        pc = s[b + 1]
        if label == 'service':
            if s[TOK] != s[HQW]:
                raise Violation('body-without-token', 'a waiting body runs while %s holds the token'
                                % (self.names[s[TOK]] if s[TOK] else 'nobody'))
            self.touch(s, a, 'group')
            s[HQ] = 2
            note('services its host queue: runs %s\'s waiting body under its token'
                 % self.names[s[HQW]])
            return
        if label == 'take_failed':
            s[Q] = a
            self.touch(s, a, 'rec')
            s[FLIST] = 0
            note('take_failed: takes Q, takes g off the failed list; detaches it')
            s[b + 1] = D_2
            s[b + 2] = AUX_TF
            return
        if pc == A_IDLE:   # label 'op'
            op = self.ops[opi]
            if op == 'body':
                if s[ATT]:
                    note('starts a host body')
                    s[b + 1] = H_Q
                else:
                    note('a host body: the group was detached; detach ran it')
                    s[b] += 1
            elif s[ATT]:
                note('starts the %s' % op)
                s[b + 1] = D_0
                s[b + 2] = AUX_TEARDOWN if op == 'teardown' else 0
            elif op == 'teardown':
                note('starts the teardown of the detached group')
                s[b + 1] = D_U
                s[b + 2] = AUX_TEARDOWN
            else:
                note('detach: not attached: nothing')
                s[b] += 1
                s[b + 1] = A_DONE
            return
        if pc == A_ATT_OR:
            self.touch(s, a, 'word')
            old = s[WORD]
            s[WORD] |= S | N
            if old & (S | D):
                note('attach: installs the notifier; wakes g: fetch_or(S|N) finds %s: done'
                     % wstr(old))
                s[b] = 1
                s[b + 1] = A_IDLE
            else:
                note('attach: installs the notifier; wakes g: fetch_or(S|N) finds %s' % wstr(old))
                s[b + 1] = A_ATT_Q
        elif pc == A_ATT_Q:
            s[Q] = a
            note('takes Q')
            s[b + 1] = A_ATT_QUEUE
        elif pc == A_ATT_QUEUE:
            if s[WORD] & D:
                note('DETACHING: does not queue')
            else:
                self.queue(s, a)
                note('queues g')
            s[b + 1] = A_ATT_REL
        elif pc == A_ATT_REL:
            s[Q] = 0
            note('releases Q')
            s[b] = 1
            s[b + 1] = A_IDLE
        elif pc < 40:
            self.step_host(s, a, b, label, note)
        else:
            self.step_detach(s, a, b, pc, note)

    def step_detach(self, s, a, b, pc, note):
        aux = s[b + 2]
        if pc == D_0:
            s[Q] = a
            note('detach step 1: takes Q')
            s[b + 1] = D_1
        elif pc == D_1:
            self.touch(s, a, 'word')
            s[WORD] |= D
            note('fetch_or(D)')
            s[b + 1] = D_2
        elif pc == D_2:
            s[Q] = 0
            note('releases Q')
            s[b + 1] = D_3
        elif pc == D_3:
            s[Q] = a
            self.touch(s, a, 'rec')
            note('step 2: takes Q: PUMPING clear, no host_held of another thread')
            s[b + 1] = D_4
        elif pc == D_4:
            s[Q] = 0
            note('releases Q')
            s[b + 1] = D_5
        elif pc == D_5:
            self.touch(s, a, 'group')
            s[TOK] = a
            s[TOKD] += 1
            note('step 3: takes the token')
            s[b + 1] = D_6 if aux & AUX_TEARDOWN else D_7
        elif pc == D_6:
            s[CLOSED] = 1
            note('closes the lock port (under PM)')
            s[b + 1] = D_7
        elif pc == D_7:
            s[Q] = a
            note('step 4: takes Q')
            s[b + 1] = D_8
        elif pc == D_8:
            self.touch(s, a, 'rec')
            s[QN] = 0
            s[HEAP] = 0
            s[FLIST] = 0
            s[LEFT] = 1
            note('leaves the ready queue, the failed list and the timer heap')
            s[b + 1] = D_9
        elif pc == D_9:
            s[Q] = 0
            note('releases Q')
            s[b + 1] = D_10
        elif pc in (D_10, D_11, D_12):
            act = self.tail[pc - D_10]
            if act == 'clear':
                s[NTF] = 0
                s[CLEARED] = 1
                note('clears the notifier (under NL)')
            elif act == 'drop':
                s[ATT] = 0
                note('drops the attachment (the pool\'s record)')
            else:
                s[WORD] = 0
                note('the word goes to 0')
            s[b + 1] = pc + 1
        elif pc == D_13:
            if aux & AUX_TEARDOWN:
                s[DESTROYED] = 1
                s[TOK] = 0
                s[TOKD] = 0
                note('step 5: kills, shuts the host down, purges, reaps: the group is destroyed')
                s[b] += 1
                s[b + 1] = A_DONE
                return
            self.release_token(s, a)
            note('step 5: runs the group\'s host-queue entries, releases the token')
            if aux & AUX_NESTED:
                s[b + 1] = H_TREL
            elif aux & AUX_TF:
                s[b + 1] = A_IDLE
                s[b + 2] = 0
            else:
                s[b] += 1
                s[b + 1] = A_DONE
        elif pc == D_U:
            s[TOK] = a
            s[TOKD] += 1
            note('takes the token')
            s[b + 1] = D_13

    # ------------------------------------------------------------------------------------------

    def describe_state(self, s):
        t = s[TOK]
        return ('word=%s queued=%d host_held=%s deferred=%d token=%s flags=%d ready=%d%s%s'
                % (wstr(s[WORD]), s[QN], self.names[s[HH]] if s[HH] else '-', s[DFR],
                   self.names[t] if t else '-', s[FLAGS], s[READY],
                   ' sleeper heap=%d' % s[HEAP] if s[SLEEPER] or s[HEAP] else '',
                   '' if s[ATT] else ' detached'))


POST_NAMES = {R_START: 'takes IM or PM, pushes', R_NL: 'takes NL', R_OR: 'wake: fetch_or(S|N)',
              R_Q: 'wake: takes Q', R_QUEUE: 'wake: queues g', R_QREL: 'wake: releases Q',
              R_END: 'releases NL'}
PC_NAMES = {
    'worker': {W_IDLE: 'pops g', W_RELQ0: 'releases Q', W_PREL: 'releases Q',
               W_CLRN: 'fetch_and(~N)', W_TOK: 'takes the token', W_RUN: 'drains and runs',
               W_WAITB: 'ends the run', W_TREL: 'releases the token', W_Q2: 'takes Q',
               W_CLRP: 'release: fetch_and(~P)', W_FAIL: 'failure: fetch_or(D)',
               W_CAS: 'release: CAS(S -> 0)', W_STORE: 'stores 0', W_REL: 'releases Q'},
    'timer': {T_IDLE: 'fires', T_OR: 'as wake: fetch_or(S|N)', T_REL: 'releases Q'},
    'host': {H_START: 'with_group', H_Q: 'takes Q, sets host_held', H_QREL: 'releases Q',
             H_TOK: 'takes the token', H_RUN: 'runs fn', H_TREL: 'releases the token',
             H_Q2: 'takes Q', H_CLR: 'clears host_held', H_WOR: 'as wake: fetch_or(S|N)',
             H_REL: 'releases Q', H_QRELU: 'releases Q', H_U: 'takes the token (unattached)',
             H_URUN: 'runs fn', H_UREL: 'releases the token'},
    'attacher': {A_IDLE: 'starts its next op', A_ATT_OR: 'attach: fetch_or(S|N)',
                 A_ATT_Q: 'takes Q', A_ATT_QUEUE: 'queues g', A_ATT_REL: 'releases Q',
                 D_0: 'detach step 1: takes Q', D_1: 'fetch_or(D)', D_2: 'releases Q',
                 D_3: 'step 2: takes Q', D_4: 'releases Q', D_5: 'step 3: takes the token',
                 D_6: 'closes the lock port', D_7: 'step 4: takes Q', D_8: 'leaves the queues',
                 D_9: 'releases Q', D_10: 'step 4, outside Q', D_11: 'step 4, outside Q',
                 D_12: 'step 4, outside Q', D_13: 'step 5', D_U: 'takes the token'},
}


def pc_name(m, s, a):
    """What actor `a`'s next step in `s` is (a trace's violating step)."""
    k = m.kind[a]
    b = m.base(a)
    if k == 'poster':
        return POST_NAMES[s[b] or R_START]
    pc = s[b + 1] if k == 'attacher' else s[b]
    if k in ('host', 'attacher') and H_POST < pc < H_DONE:
        return POST_NAMES[pc - H_POST]
    if k == 'attacher' and pc < 40 and pc != A_IDLE:
        k = 'host'
    return PC_NAMES[k].get(pc, str(pc))


def label_str(label):
    if isinstance(label, tuple):
        if label[0] == 'ok':
            return 'ok(runnable=%d,deadline=%s)' % (label[1], ('none', 'future', 'due')[label[3]])
        return label[0]
    return label


def explore(cfg, max_states=None, log=None, symmetry=True):
    """Every reachable state, breadth first (with `symmetry`, one state per orbit of the workers'
    permutations). Returns a dict: states, transitions, idle_states (the idle states checked),
    seconds, complete, violations {name: {detail, steps, trace}}."""
    m = Model(cfg)
    canon = m.canon if symmetry else (lambda s: (bytes(s), None))
    s0 = m.initial()
    k0 = canon(s0)[0]
    parent = {k0: None}
    dq = collections.deque([k0])
    viol = {}
    transitions = 0
    idle = 0
    t0 = time.time()

    def found(name, detail, k, act):
        if name not in viol:
            viol[name] = (detail, k, act)

    while dq:
        if max_states and len(parent) > max_states:
            break
        k = dq.popleft()
        s = list(k)
        enabled = []
        for a in m.actors:
            for lab in m.options(s, a):
                enabled.append((a, lab))
        if m.idle(s):
            idle += 1
            try:
                m.idle_check(s)
            except Violation as v:
                found(v.name, v.detail, k, None)
        elif not enabled:
            found('deadlock', 'no step is enabled: ' + m.describe_state(s), k, None)
        for a, lab in enabled:
            s2 = list(s)
            transitions += 1
            try:
                m.apply(s2, a, lab)
                m.invariants(s2)
            except Violation as v:
                found(v.name, v.detail, k, (a, lab))
                continue
            k2 = canon(s2)[0]
            if k2 not in parent:
                parent[k2] = (k, (a, lab))
                dq.append(k2)
                if log and len(parent) % 500000 == 0:
                    log('%d states, %d transitions, %.0fs' % (len(parent), transitions,
                                                              time.time() - t0))
    res = {'config': cfg.describe(), 'states': len(parent), 'transitions': transitions,
           'idle_states': idle, 'seconds': round(time.time() - t0, 1), 'complete': not dq,
           'violations': {}}
    for name, (detail, k, act) in sorted(viol.items()):
        path = []
        h = k
        while parent[h] is not None:
            ph, pa = parent[h]
            path.append(pa)
            h = ph
        path.reverse()
        if act is not None:
            path.append(act)
        trace, real = render(m, path)
        res['violations'][name] = {'detail': real or detail, 'steps': len(path), 'trace': trace}
    return res


def render(m, path):
    """Replay `path` (each action named as in the canonical form of the state it leaves) on the
    real state, naming the real actors. Returns (lines, the violation's detail or None)."""
    s = m.initial()
    perm = m.canon(s)[1]
    out = ['initial: ' + m.describe_state(s)]
    for a, lab in path:
        a = m.real_actor(a, perm)
        notes = []
        what = pc_name(m, s, a)
        try:
            m.apply(s, a, lab, notes)
            m.invariants(s)
        except Violation as v:
            out.append('%s: %s  -> VIOLATION %s: %s' % (m.names[a], '; '.join(notes) or what,
                                                       v.name, v.detail))
            return out, v.detail
        out.append('%s: %s  [%s]' % (m.names[a], '; '.join(notes) or label_str(lab),
                                     m.describe_state(s)))
        perm = m.canon(s)[1]
    if m.idle(s):
        try:
            m.idle_check(s)
        except Violation as v:
            out.append('every actor is idle or done -> VIOLATION %s: %s' % (v.name, v.detail))
            return out, v.detail
    out.append('-> DEADLOCK: no step is enabled: ' + m.describe_state(s))
    return out, 'no step is enabled: ' + m.describe_state(s)


# ----------------------------------------------------------------------------------------------
# configurations and the CI suites

def parse_kv(text):
    out = {}
    if text:
        for part in text.split(','):
            k, _, v = part.partition('=')
            out[k.strip()] = int(v)
    return out


def make_config(actors=None, opts=None, amend=(), mutation=()):
    a = dict(posters=2, workers=2, hosts=1, bodies=1, teardown=1, detach=0)
    a.update(actors or {})
    o = dict(posts=1, timer=1, sleeps=1, waiting=0, failures=0, pre=0, nested=0)
    o.update(opts or {})
    end = 'teardown' if a.pop('teardown') else ('detach' if a.pop('detach') else None)
    a.pop('detach', None)
    return Config(posters=a['posters'], workers=a['workers'], hosts=a['hosts'],
                  bodies=a['bodies'], end=end, amend=amend, mutation=mutation, **o)


# Each check: (name, actors, opts, amend, mutation, must, allowed): the violations found must
# include every name in `must` and be among `must` and `allowed` (a check expecting nothing has
# both empty). The frozen protocol's findings are each isolated by every amendment but the one
# that fixes it, on the smallest configuration that shows it.
ALL = 'all'
BUT = lambda name: tuple(a for a in AMENDMENTS if a != name)  # noqa: E731
QUICK = [
    # every amendment: nothing found
    ('amended', {}, {}, ALL, (), (), ()),
    ('amended-detach', dict(posters=1, hosts=2, bodies=0, teardown=0, detach=1), dict(failures=1),
     ALL, (), (), ()),
    ('amended-waiting', dict(posters=1, hosts=0), dict(waiting=1, failures=1), ALL, (), (), ()),
    ('amended-nested', dict(posters=1, workers=1, hosts=0, teardown=0, detach=1),
     dict(timer=0, nested=1), ALL, (), (), ()),
    # each finding of the frozen transitions (README.md), without its amendment
    ('finding-host-wake', dict(posters=0, workers=1, bodies=0, teardown=0), dict(timer=0),
     BUT('host-wake'), (), ('lost-wake',), ()),
    ('finding-host-excl', dict(posters=0, workers=1, teardown=0), dict(timer=0), BUT('host-excl'),
     (), ('token-contended',), ('pump-host-held',)),
    ('finding-host-excl-teardown', dict(posters=0, workers=1), dict(timer=0), BUT('host-excl'), (),
     ('token-contended', 'use-after-free'), ('pump-host-held', 'record-after-detach')),
    ('finding-host-detaching', dict(posters=0, workers=1, bodies=0, teardown=0, detach=1),
     dict(timer=0), BUT('host-detaching'), (), ('record-after-detach', 'token-contended'), ()),
    ('finding-nested', dict(posters=1, workers=1, hosts=0, teardown=0, detach=1),
     dict(timer=0, nested=1), BUT('nested-release'), (), ('record-after-detach',), ()),
    # the mutations, on the amended protocol
    ('mutation-release-store', dict(posters=1, workers=1, hosts=0, bodies=0, teardown=0), {}, ALL,
     ('release-store',), ('lost-wake',), ()),
    ('mutation-pump-host-held', dict(posters=1, workers=1, bodies=0, teardown=0), {}, ALL,
     ('pump-host-held',), ('pump-host-held',), ('token-contended',)),
    ('mutation-zero-before-clear', dict(posters=1, workers=1, hosts=0, bodies=0), {}, ALL,
     ('zero-before-clear',), ('read-after-detach',), ('queued-unscheduled', 'use-after-free')),
    ('mutation-wake-no-recheck', dict(posters=1, workers=1, hosts=0, bodies=0), {}, ALL,
     ('wake-no-recheck',), ('read-after-detach',), ('use-after-free',)),
    ('mutation-pop-no-detaching', dict(posters=1, workers=1, hosts=0, bodies=0), {}, ALL,
     ('pop-no-detaching',), ('read-after-detach',), ('token-contended', 'use-after-free')),
]
DEEP = [
    # the frozen transitions, and each finding alone, at the default bound: nothing else found
    ('deep-frozen', {}, {}, (), (), ('lost-wake', 'token-contended'),
     ('pump-host-held', 'record-after-detach', 'use-after-free')),
    ('deep-finding-host-wake', {}, {}, BUT('host-wake'), (), ('lost-wake',), ()),
    ('deep-finding-host-excl', {}, {}, BUT('host-excl'), (), ('token-contended',),
     ('pump-host-held', 'record-after-detach', 'use-after-free')),
    ('deep-finding-host-detaching', dict(teardown=0, detach=1), {}, BUT('host-detaching'), (),
     ('record-after-detach', 'token-contended'), ()),
    ('deep-finding-nested', dict(teardown=0, detach=1), dict(nested=1), BUT('nested-release'), (),
     ('record-after-detach',), ()),
    # every amendment at larger bounds: nothing found
    ('deep-nested', dict(teardown=0, detach=1), dict(nested=1), ALL, (), (), ()),
    ('deep-failures-waiting', {}, dict(failures=1, waiting=1), ALL, (), (), ()),
    ('deep-three-workers', dict(workers=3), {}, ALL, (), (), ()),
    ('deep-two-posts', {}, dict(posts=2, pre=1), ALL, (), (), ()),
    ('deep-detach', dict(hosts=2, teardown=0, detach=1), dict(failures=1), ALL, (), (), ()),
    ('deep-two-hosts', dict(hosts=2), dict(failures=1, waiting=1), ALL, (), (), ()),
    ('deep-three-posters', dict(posters=3, hosts=2), dict(failures=1), ALL, (), (), ()),
]


def say(msg):
    print(msg, flush=True)


def run_check(check, out=say):
    """Run one suite check; True when it holds."""
    name, actors, opts, amend, mutation, must, allowed = check
    r = explore(make_config(actors, opts, amend, mutation))
    got = set(r['violations'])
    ok = r['complete'] and set(must) <= got <= set(must) | set(allowed)
    out('%-28s %s %9d states %10d transitions %7.1fs  found %s%s'
        % (name, 'ok  ' if ok else 'FAIL', r['states'], r['transitions'], r['seconds'],
           ', '.join(sorted(got)) or 'nothing',
           '' if ok else '  (expected %s)' % (', '.join(must) or 'nothing')))
    if not ok:
        for vn, v in sorted(r['violations'].items()):
            out('  %s: %s' % (vn, v['detail']))
            for line in v['trace']:
                out('    ' + line)
    return ok


def run_suite(checks, out=say):
    t0 = time.time()
    bad = sum(not run_check(c, out) for c in checks)
    out('state_word.py: %d checks, %d unexpected, %.0fs' % (len(checks), bad, time.time() - t0))
    return bad


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--actors', default='')
    ap.add_argument('--opts', default='')
    ap.add_argument('--amend', default='all')
    ap.add_argument('--mutation', default='')
    ap.add_argument('--max-states', type=int, default=0)
    ap.add_argument('--suite', choices=('quick', 'deep'))
    args = ap.parse_args(argv)
    if args.suite:
        checks = QUICK if args.suite == 'quick' else QUICK + DEEP
        return 1 if run_suite(checks) else 0
    amend = ALL if args.amend == 'all' else tuple(
        x for x in args.amend.split(',') if x and x != 'none')
    mutation = tuple(x for x in args.mutation.split(',') if x)
    cfg = make_config(parse_kv(args.actors), parse_kv(args.opts), amend, mutation)
    r = explore(cfg, args.max_states or None,
                log=lambda msg: print(msg, file=sys.stderr, flush=True))
    print(json.dumps(r, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
