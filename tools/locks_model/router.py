"""The router protocol across exec groups (S2b; S2.md S2-Q10, S2-Q15; td 4.6, 4.11, 8.4 MS).

An exhaustive model of the protocol S2b.0 froze in the header comment of
`inc/cvc/state/state_exec/exec_router.h` ("Islands", "The protocol", "Broadcasts", "The cap",
"Teardown and rehoming", the throw rule of "The ordered ingress"), on one resolved channel key
homed at a group H that registered its prefix, and at group 0 (G0) once H's teardown unregisters
it. `--amend none` is S2b.0's text; `--amend all` (the default) is S2b.0a's, the text as amended
from this model's findings. Like state_word.py it has no C++ port. Standard library only.

The groups are G0, H, R1, R2 and S, and X for host threads (exec_pool::post_message, which posts
to the key's home and is neither killed nor torn down). Each group's thread is one actor, and each
step it takes is
atomic: it applies the head of its ordered ingress, steps one of its processes, fires one
timeout, applies one kill, or takes one step of its teardown. A post is an atomic append to the
target's ingress (group_ref::post); a closed handle refuses it, and a poster re-resolves the home
and posts again (a home-bound entry), so a post races every other group's steps. The lock port,
completions and controls other than kills are out of scope. A timed wait's deadline passes at
any moment (a clock step, `expire`), after which its home skips its record and its group may fire
the timeout. Every interleaving is explored, breadth first, so each trace is a shortest one.

The processes run scripts: `r` a msg-recv, `t` a msg-recv with a deadline, `s` a msg-send of a new
message, `p` (X only) a host's post_message of a new message (each `s` or `p` is its own message,
m1, m2, ... in the order of the configuration). A process
whose group is the key's home receives locally (the pending FIFO's front, else it parks as a
local waiter); elsewhere it parks with a new recv_seq and posts `subscribe` to the home. Kills
(`kills=N`, any process, at any time) and timeouts post `unsubscribe` for a remote wait. The
entries and their effects are route_entry's kinds as the header gives them: send, subscribe,
unsubscribe, deliver (by pid and seq; stale: a requeue, or dropped when requeue_if_stale is
false), requeue (the next waiter, else the front of the FIFO, past its cap), taken, adopt (S2b.0a),
and a host's post (a message entry, capped, applied as a local send); broadcast
tracking (no local taker and k >= 2 live records: tracked by an id; one record: untracked; a
local taker: the remote copies dropped when stale); the ingress cap (`icap`, capped sends only)
and the pending FIFO's cap (max_pending_messages, a per-scheduler field: `fcap` H's, `fcap0`
G0's); H's teardown (`teardown=1`) and a receiving group's (`rdie=1`, R1's), each as the header
orders them; a deliver that throws before it patches its frame (`throws=1`); other traffic at G0
(`busy0=1`: a capped send on another key in its ingress at the start, so the cap can refuse a
forward); the requeue's next waiter chosen every way the text allows (`choice=1`); and when the
router is on (exec_router::active()): R1, R2 and S each register a prefix of their own, not
covering the key, so it is on throughout, unless `islands=1`, where they register none and it is
on only while H is registered or a record exists. In S2b.0's text a process's msg-send and
msg-recv stay in its own scheduler while the router is off (today's path, RT-1); with
`island-rule` an island keeps the key in its own scheduler once no other group's prefix covers it
(H gone), router on or off.

Properties, each a violation name:
- at a receipt: `duplicate` (a message reaches a second wait: one it was pending for, or one
  outside the waiters its send found at the home once one of those took it), `misaddressed` (a
  deliver patches a wait other than the one it names);
- at a tracker's creation or adoption, and at an answer: `id-collision` (a home tracks two
  broadcasts under one id, or an answer is applied to another message's tracker);
- at a message's first assignment to a wait (a local receipt, or a deliver posted): `fifo` (an
  earlier message of the same sender group, to the key, is not yet assigned: td 4.11's FIFO per
  (sender group, channel), as the home's order of handing messages out);
- at idle states (every ingress empty, no teardown under way, no process runnable): `lost` (a
  message neither received nor pending; a fresh send that a full FIFO dropped is today's
  behaviour, counted, and allowed), `duplicate` (pending twice, or pending and received),
  `stranded` (a message pending where a wait not past its deadline is served, or an island's own
  message, sent once H was gone, pending elsewhere while that island waits locally),
  `orphan-wait` (such a wait not registered at its home: no local waiter, no record),
  `stale-record` (a record at the home for a wait that ended), `tracker-leak` (a tracked
  broadcast still waiting for an answer);
- `deadlock`: no step enabled while not idle; `nondeterministic` (`choice=1`): an internal choice
  the text leaves open (the requeue's "next waiter") changes the outcome of one step.
The documented gaps of exec_router.h and td 4.11 are reported under their own names and do not
stop the exploration: `gap-requeue-order` (a requeued message handed out after a later message
of its sender) and `gap-island-rehomed` (with islands, once H is gone: a message pending in one
place while a wait is served in another, the islands keeping the key to themselves and the waits
parked at H rehomed to G0).

`--amend` adds the amendments, as S2b.0a took them (README.md, "The router model"):
- `rehome-order`: the teardown closes its handle first, forwards, and only then unregisters; a
  poster the closed handle refuses waits while the home is still that group (td 4.11's FIFO and no
  stale records across a teardown; an adopt ahead of every late answer);
- `forward-pending-first`: the pending messages are forwarded first (they were applied before
  anything still queued), then the records, then the trackers (adopt), then the queued entries;
- `forward-uncapped`: a queued send the teardown forwards is uncapped, like a pending one;
- `uncapped-past-cap`: an uncapped send that finds no waiter joins the FIFO past its cap;
- `stale-rule`: a deliver the teardown or a throw sends back follows requeue_if_stale (dropped
  when false), as a stale one does;
- `throw-resubscribe`: a deliver that threw before patching also posts a subscribe for the wait
  (the same seq): the process still waits, and its record was used;
- `refused-stale`: a deliver its receiver's closed handle refuses is a stale copy at the home
  (requeue_if_stale's rule, applied there at once);
- `queue-as-requeue`: a tracked broadcast whose copies all came back stale is applied as a
  requeue (the next waiter, else the front of the FIFO), not queued past a waiter;
- `unique-ids`: broadcast ids are unique in the app, so an answer that reaches another home after
  a teardown never names that home's own broadcast;
- `next-waiter-order`: "the next waiter" is fixed (the earliest local waiter, else the earliest
  record), so a deterministic schedule has one outcome;
- `island-rule`: a group with no prefix keeps every key no other group's prefix covers in its own
  scheduler, router on or off (S2b.0a's answer to the finding that its messaging changed home as
  the router turned on and off); it homes nothing for another group, so its teardown forwards
  none of its own pending messages: they die with it, counted, as a Stage 1 group's do (expected,
  not `lost`);
- `adopt`: a teardown forwards each broadcast tracker it holds (an `adopt` entry: value, id,
  copies outstanding, taken), which the new home takes over, so the copies' late answers find it
  (it needs rehome-order's wait and unique-ids);
- `forward-host-posts`: a host post queued at a dying home is forwarded as an uncapped send, in
  queue order (S2b.0 says nothing; read as "its queued router entries" only, it dies there).

`--mutation` breaks one rule, to show the checks find it: `no-requeue` (a stale deliver is always
dropped), `requeue-always` (a stale copy is requeued although a local waiter took the message),
`untracked` (k >= 2 remote copies untracked), `no-unsubscribe` (a timeout or kill posts no
unsubscribe), `subscribe-no-fifo` (a subscribe always records, never answering from the FIFO),
`deliver-no-seq` (a deliver patches the process's current wait whatever its seq).

usage:
    python -I router.py [--procs S:ss,R1:t,R2:r] [--opts kills=1,teardown=1,rdie=0,throws=0,
                         icap=0,fcap=0,fcap0=0,choice=0,busy0=0,islands=0]
                        [--amend all|none|NAME,... | --but NAME,...]
                        [--mutation NAME]
    python -I router.py --suite quick|deep      the CI checks (README.md); exit 0 when all hold

`--procs` lists processes as GROUP:SCRIPT (groups G0, H, R1, R2, S, X); `--but` runs every amendment
but the ones named. The JSON printed has `states`, `transitions`, `idle_states`, `seconds`,
`complete`, `peak_mb` and `violations`, each with its detail, its length in steps and its trace.
"""

import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))

import argparse  # noqa: E402
import array  # noqa: E402
import collections  # noqa: E402
import io  # noqa: E402
import json  # noqa: E402
import pickle  # noqa: E402
import sys  # noqa: E402
import time  # noqa: E402

from lockmodel import Violation  # noqa: E402

G0, H, R1, R2, S, X = range(6)
GROUPS = ('G0', 'H', 'R1', 'R2', 'S', 'X')   # X: host threads (exec_pool::post_message)
NG = 6

AMENDMENTS = ('rehome-order', 'forward-pending-first', 'forward-uncapped', 'uncapped-past-cap',
              'stale-rule', 'throw-resubscribe', 'refused-stale', 'queue-as-requeue',
              'unique-ids', 'next-waiter-order', 'island-rule', 'adopt', 'forward-host-posts')
MUTATIONS = ('no-requeue', 'requeue-always', 'untracked', 'no-unsubscribe', 'subscribe-no-fifo',
             'deliver-no-seq')
GAPS = ('gap-requeue-order', 'gap-island-rehomed')

# a process: (pc, status, seq, expired-seqs bitmask)
RUN, WL, WR, DONE, DEAD = range(5)
STATUS = ('runs', 'waits locally', 'waits remotely', 'done', 'dead')
# a message: (status, W, receipts in W, receipts outside W, flags); W is None until the send is
# first applied at a home, then the waits it found there (empty: it went pending, one taker)
M_UNSENT, M_SENT, M_REFUSED = 0, 1, 2
F_ASSIGNED, F_DROPPED, F_REQUEUED, F_ORPHAN, F_RDROP, F_LATE, F_DIED = 1, 2, 4, 8, 16, 32, 64
# a home's state for the key: (local waiters, records, pending FIFO, trackers, next id); a record
# is (group, pid, seq, timed), a tracker (id, message, copies outstanding, taken)
EMPTY_HOME = ((), (), (), (), 1)
# teardown phases: not started, unregistered (the frozen order), forwarding, forwarded (unregister
# next, rehome-order), killing, done
TD_NONE, TD_UNREG, TD_FWD, TD_FWDDONE, TD_KILL, TD_DONE = range(6)

# the state's slots
REG, CLOSED, ING, HOMES, PROCS, MSGS, SENT, TDH, TDR, BUDGET = range(10)


class Blocked(Exception):
    """The step posts to a home whose closed handle refuses it while it is still the home
    (rehome-order): the poster waits, so the step is not enabled."""


class Config(object):
    def __init__(self, procs, kills=0, teardown=1, rdie=0, throws=0, icap=0, fcap=0, fcap0=0,
                 choice=0, busy0=0, islands=0, amend=(), mutation=()):
        self.procs = tuple((GROUPS.index(g), ops) for g, ops in procs)
        for g, ops in self.procs:
            assert ops and set(ops) <= (set('p') if g == X else set('rts')), (GROUPS[g], ops)
        self.kills = kills
        self.teardown = teardown
        self.rdie = rdie
        self.throws = throws
        self.icap = icap
        self.fcap = fcap          # H's max_pending_messages (0: none)
        self.fcap0 = fcap0        # G0's (a per-scheduler field: the two may differ)
        self.choice = choice
        self.busy0 = busy0        # G0's ingress starts with a capped send on another key
        self.islands = islands    # R1, R2 and S register no prefix (else each its own, elsewhere)
        self.amend = frozenset(AMENDMENTS if amend == 'all' else amend)
        self.mutation = frozenset(mutation)
        for a in self.amend:
            assert a in AMENDMENTS, a
        for m in self.mutation:
            assert m in MUTATIONS, m
        # the messages: one per `s` or `p`, numbered in order, with their sender
        self.msg_proc = []
        self.op_msg = {}
        for p, (_, ops) in enumerate(self.procs):
            for i, op in enumerate(ops):
                if op in 'sp':
                    self.op_msg[(p, i)] = len(self.msg_proc)
                    self.msg_proc.append(p)

    def describe(self):
        return {'procs': ['%s:%s' % (GROUPS[g], ops) for g, ops in self.procs],
                'kills': self.kills, 'teardown': self.teardown, 'rdie': self.rdie,
                'throws': self.throws, 'icap': self.icap, 'fcap': self.fcap, 'fcap0': self.fcap0,
                'choice': self.choice, 'busy0': self.busy0, 'islands': self.islands,
                'amend': sorted(self.amend),
                'mutation': sorted(self.mutation)}


def router_on(cfg, s):
    """exec_router::active(): a group other than group 0 has a registered prefix (R1, R2 and S
    each have one of their own unless `islands`; H while registered), or a record exists at some
    home."""
    if not cfg.islands:
        return True
    return bool(s[REG]) or any(hs[1] for hs in s[HOMES])


def pname(cfg, p):
    return 'p%d@%s' % (p + 1, GROUPS[cfg.procs[p][0]])


def mname(m):
    return 'm%d' % (m + 1)


def bname(b):
    return 'b%d' % b if isinstance(b, int) else 'b%s.%d' % (GROUPS[b[0]], b[1])


class Work(object):
    """One step's working copy of a state: the slots it writes are replaced, never mutated."""

    def __init__(self, model, s, oracle=(), notes=None):
        self.m = model
        self.cfg = model.cfg
        self.am = model.cfg.amend
        self.mut = model.cfg.mutation
        self.s = list(s)
        self.notes = notes
        self.oracle = oracle
        self.taken = []
        self.points = []
        self.soft = []

    def note(self, text):
        if self.notes is not None:
            self.notes.append(text)

    def freeze(self):
        return tuple(self.s)

    # --- slots ---
    def set_item(self, slot, i, v):
        t = list(self.s[slot])
        t[i] = v
        self.s[slot] = tuple(t)

    def proc(self, p):
        return self.s[PROCS][p]

    def set_proc(self, p, v):
        self.set_item(PROCS, p, v)

    def msg(self, m):
        return self.s[MSGS][m]

    def set_msg(self, m, v):
        self.set_item(MSGS, m, v)

    def flag(self, m, f):
        st, w, rin, rout, fl = self.msg(m)
        self.set_msg(m, (st, w, rin, rout, fl | f))

    def home(self, h):
        return self.s[HOMES][h]

    def set_home(self, h, v):
        self.set_item(HOMES, h, v)

    def closed(self, g):
        return self.s[CLOSED][g]

    def homeg(self):
        return H if self.s[REG] else G0

    def local_home(self, g):
        """Where a msg-send or msg-recv of a process in g goes: the key's home while the router
        is on, else g itself (today's path, one scheduler at a time: RT-1). With island-rule, an
        island (no prefix) keeps the key while no other group's prefix covers it (H gone), router
        on or off."""
        if self.cfg.islands and g in (R1, R2, S):
            if 'island-rule' in self.am:
                return H if self.s[REG] else g
            if not router_on(self.cfg, self.s):
                return g
        return self.homeg()

    def choose(self, n):
        i = len(self.taken)
        c = self.oracle[i] if i < len(self.oracle) else 0
        self.taken.append(c)
        self.points.append(n)
        return c

    def expired(self, rec):
        return rec[3] and (self.proc(rec[1])[3] >> rec[2]) & 1

    # --- posts ---
    def push(self, g, e):
        """Append `e` to g's ingress; False when a capped send finds the ingress at its cap."""
        q = self.s[ING][g]
        capped = (e[0] == 'S' and not e[2]) or e[0] == 'P'
        if capped and self.cfg.icap:
            if sum(1 for x in q if (x[0] == 'S' and not x[2]) or x[0] == 'P') >= self.cfg.icap:
                return False
        self.set_item(ING, g, q + (e,))
        return True

    def post_home(self, e):
        """A home-bound entry (send, subscribe, unsubscribe) to the key's home, re-resolved."""
        h = self.homeg()
        if self.closed(h):
            raise Blocked()   # only with rehome-order: H closed while still registered
        ok = self.push(h, e)
        self.note('posts %s to %s%s' % (estr(self.cfg, e), GROUPS[h], '' if ok else
                                         ': refused (ingress full)'))
        return ok

    def post_from(self, e, frm):
        """A requeue or taken to the deliver's home `frm`; its closed handle refuses it and the
        poster re-resolves the home."""
        if not self.closed(frm):
            self.push(frm, e)
            self.note('posts %s to %s' % (estr(self.cfg, e), GROUPS[frm]))
            return
        h = self.homeg()
        if self.closed(h):
            raise Blocked()
        self.push(h, e)
        self.note('posts %s to %s: refused, re-resolves to %s' % (estr(self.cfg, e), GROUPS[frm],
                                                                  GROUPS[h]))

    def post_deliver(self, g, e):
        """A home posts a deliver to the receiving group g."""
        if not self.closed(g):
            self.push(g, e)
            self.note('posts %s to %s' % (estr(self.cfg, e), GROUPS[g]))
            return
        _, frm, p, seq, m, bid, rqis = e
        if 'refused-stale' in self.am:
            self.note('posts %s to %s: refused: a stale copy at once' % (estr(self.cfg, e),
                                                                         GROUPS[g]))
            if rqis:
                self.apply_requeue(frm, m, bid)
            return
        # the frozen text says nothing about a deliver a closed receiver refuses
        self.note('posts %s to %s: refused (closed); nothing more is said: dropped'
                  % (estr(self.cfg, e), GROUPS[g]))

    # --- messages ---
    def mark_sent(self, g, m):
        st, w, rin, rout, fl = self.msg(m)
        self.set_msg(m, (M_SENT, w, rin, rout, fl | (0 if self.s[REG] else F_LATE)))
        self.set_item(SENT, g, self.s[SENT][g] + (m,))

    def assign(self, m):
        """m is handed to a wait (a receipt, or a deliver posted): FIFO per (sender group, key)."""
        st, w, rin, rout, fl = self.msg(m)
        g = self.cfg.procs[self.cfg.msg_proc[m]][0]
        order = self.s[SENT][g]
        i = order.index(m)
        if not fl & F_ASSIGNED:
            self.set_msg(m, (st, w, rin, rout, fl | F_ASSIGNED))
            for m2 in order[:i]:
                if not self.msg(m2)[4] & (F_ASSIGNED | F_DROPPED | F_DIED):
                    raise Violation('fifo', '%s is handed out before %s, sent earlier by %s'
                                    % (mname(m), mname(m2), GROUPS[g]))
        else:
            for m2 in order[i + 1:]:
                if self.msg(m2)[4] & F_ASSIGNED:
                    self.soft.append(('gap-requeue-order',
                                      '%s, requeued, is handed out again after %s, sent later by %s'
                                      % (mname(m), mname(m2), GROUPS[g])))
                    break

    def receive(self, m, p):
        """p's current wait takes m."""
        pc, st, seq, exp = self.proc(p)
        mst, w, rin, rout, fl = self.msg(m)
        assert w is not None
        if (p, seq) in w:
            rin += 1
            dup = rout >= 1
        else:
            rout += 1
            dup = rin >= 1 or rout >= 2
        self.set_msg(m, (mst, w, rin, rout, fl))
        self.note('%s takes %s' % (pname(self.cfg, p), mname(m)))
        self.advance(p)
        if dup:
            detail = ('%s reaches a second wait (%s#%d): %d of the waits its send found, %d others'
                      % (mname(m), pname(self.cfg, p), seq, rin, rout))
            if fl & F_ORPHAN:
                detail += ' (a broadcast its home dropped the tracking of, with copies outstanding)'
            raise Violation('duplicate', detail)

    def advance(self, p):
        pc, st, seq, exp = self.proc(p)
        pc += 1
        self.set_proc(p, (pc, DONE if pc == len(self.cfg.procs[p][1]) else RUN, seq, exp))

    # --- applied at a home ---
    def new_bid(self, h):
        hs = self.home(h)
        n = hs[4]
        self.set_home(h, hs[:4] + (n + 1,))
        return (h, n) if 'unique-ids' in self.am else n

    def apply_send(self, h, m, unc):
        if m is None:
            self.note('a send on another key')
            return
        loc, recs, pend, trs, nid = self.home(h)
        live = tuple(r for r in recs if not self.expired(r))
        st, w, rin, rout, fl = self.msg(m)
        rehomed = w is not None   # applied before: a pending message the old home forwarded
        if w is None:
            w = tuple(sorted([(p, self.proc(p)[2]) for p in loc] + [(r[1], r[2]) for r in live]))
            self.set_msg(m, (st, w, rin, rout, fl))
        if not loc and not live:
            cap = self.cfg.fcap0 if h == G0 else self.cfg.fcap
            if cap and len(pend) >= cap and not (unc and 'uncapped-past-cap' in self.am):
                self.flag(m, F_DROPPED | (F_RDROP if rehomed else 0))
                self.note('%s: no waiter, the FIFO is full: dropped (pending_dropped)' % mname(m))
                return
            self.set_home(h, (loc, recs, pend + (m,), trs, nid))
            self.note('%s: no waiter: joins %s\'s FIFO' % (mname(m), GROUPS[h]))
            return
        self.assign(m)
        rest = tuple(r for r in recs if r not in live)
        self.set_home(h, ((), rest, pend, trs, nid))
        if loc:
            for p in loc:
                self.receive(m, p)
            for r in live:   # a local waiter took it: stale copies are dropped
                self.post_deliver(r[0], ('D', h, r[1], r[2], m, 0, False))
        elif len(live) == 1 or 'untracked' in self.mut:
            for r in live:
                self.post_deliver(r[0], ('D', h, r[1], r[2], m, 0, True))
        else:
            bid = self.new_bid(h)
            hs = self.home(h)
            self.check_new_id(h, bid, m)
            self.set_home(h, hs[:3] + (hs[3] + ((bid, m, len(live), False),),) + hs[4:])
            for r in live:
                self.post_deliver(r[0], ('D', h, r[1], r[2], m, bid, True))

    def give_next(self, h, m):
        """A stale delivery back at its home: the next waiter, else the front of the FIFO."""
        self.flag(m, F_REQUEUED)
        loc, recs, pend, trs, nid = self.home(h)
        live = [r for r in recs if not self.expired(r)]
        cands = [('L', p) for p in loc] + [('R', r) for r in live]
        if not cands:
            self.set_home(h, (loc, recs, (m,) + pend, trs, nid))
            self.note('%s: no waiter: the front of %s\'s FIFO' % (mname(m), GROUPS[h]))
            return
        i = 0
        if self.cfg.choice and 'next-waiter-order' not in self.am and len(cands) > 1:
            i = self.choose(len(cands))
        kind, x = cands[i]
        self.assign(m)
        if kind == 'L':
            self.set_home(h, (tuple(p for p in loc if p != x), recs, pend, trs, nid))
            self.note('%s goes to the next waiter, %s' % (mname(m), pname(self.cfg, x)))
            self.receive(m, x)
        else:
            self.set_home(h, (loc, tuple(r for r in recs if r != x), pend, trs, nid))
            self.note('%s goes to the next waiter, %s#%d' % (mname(m), pname(self.cfg, x[1]), x[2]))
            self.post_deliver(x[0], ('D', h, x[1], x[2], m, 0, True))

    def tracker(self, h, bid):
        for i, t in enumerate(self.home(h)[3]):
            if t[0] == bid:
                return i, t
        return None, None

    def set_tracker(self, h, i, t):
        hs = self.home(h)
        trs = list(hs[3])
        if t is None:
            del trs[i]
        else:
            trs[i] = t
        self.set_home(h, hs[:3] + (tuple(trs),) + hs[4:])

    def check_new_id(self, h, bid, m):
        """A home tracks each id once: a second broadcast under an id it tracks means the ids
        collide (per-home ids, after a teardown forwarded another home's tracker)."""
        i, t = self.tracker(h, bid)
        if t is not None:
            self.soft.append(('id-collision', '%s tracks %s for %s and now for %s too'
                              % (GROUPS[h], bname(bid), mname(t[1]), mname(m))))

    def check_answer(self, h, bid, t, m):
        """An answer is for the broadcast its home tracks under that id."""
        if t[1] != m:
            self.soft.append(('id-collision', 'an answer for %s names %s, which %s tracks for %s'
                              % (mname(m), bname(bid), GROUPS[h], mname(t[1]))))

    def apply_requeue(self, h, m, bid):
        if bid:
            i, t = self.tracker(h, bid)
            if t is not None:
                self.check_answer(h, bid, t, m)
                out = t[2] - 1
                self.set_tracker(h, i, None if out == 0 else (t[0], t[1], out, t[3]))
                if t[3]:
                    self.note('%s: a stale copy of %s, already taken: dropped' % (mname(m),
                                                                                   bname(bid)))
                elif out:
                    self.note('%s: a stale copy of %s; %d outstanding' % (mname(m), bname(bid),
                                                                           out))
                elif 'queue-as-requeue' in self.am:
                    self.note('%s: every copy of %s stale: as a requeue' % (mname(m), bname(bid)))
                    self.give_next(h, m)
                else:
                    # "H queues it once": into the FIFO, past its cap
                    self.flag(m, F_REQUEUED)
                    loc, recs, pend, trs, nid = self.home(h)
                    self.set_home(h, (loc, recs, (m,) + pend, trs, nid))
                    self.note('%s: every copy of %s stale: queued once, in %s\'s FIFO'
                              % (mname(m), bname(bid), GROUPS[h]))
                return
            self.note('%s names %s, which %s does not track: untracked' % (mname(m), bname(bid),
                                                                            GROUPS[h]))
        self.give_next(h, m)

    def apply_taken(self, h, bid, m):
        i, t = self.tracker(h, bid)
        if t is None:
            self.note('%s is not tracked at %s: nothing' % (bname(bid), GROUPS[h]))
            return
        self.check_answer(h, bid, t, m)
        out = t[2] - 1
        self.set_tracker(h, i, None if out == 0 else (t[0], t[1], out, True))
        self.note('%s taken; %d outstanding' % (bname(bid), out))

    def apply_adopt(self, h, m, bid, out, taken):
        self.check_new_id(h, bid, m)
        hs = self.home(h)
        self.set_home(h, hs[:3] + (hs[3] + ((bid, m, out, taken),),) + hs[4:])
        self.note('tracks %s from here' % bname(bid))

    def apply_subscribe(self, h, g, p, seq, timed):
        loc, recs, pend, trs, nid = self.home(h)
        if pend and 'subscribe-no-fifo' not in self.mut:
            m = pend[0]
            self.set_home(h, (loc, recs, pend[1:], trs, nid))
            self.assign(m)
            self.note('answers from its FIFO')
            self.post_deliver(g, ('D', h, p, seq, m, 0, True))
            return
        self.set_home(h, (loc, recs + ((g, p, seq, timed),), pend, trs, nid))
        self.note('records %s#%d' % (pname(self.cfg, p), seq))

    def apply_unsubscribe(self, h, g, p, seq):
        loc, recs, pend, trs, nid = self.home(h)
        keep = tuple(r for r in recs if r[:3] != (g, p, seq))
        self.set_home(h, (loc, keep, pend, trs, nid))
        self.note('drops the record' if len(keep) < len(recs) else 'no record (used): nothing')

    # --- applied at a receiver ---
    def apply_deliver(self, g, e, throw=False):
        _, frm, p, seq, m, bid, rqis = e
        pc, st, cur, exp = self.proc(p)
        fresh = st == WR and (cur == seq or 'deliver-no-seq' in self.mut)
        if fresh and throw:
            self.note('the deliver throws before it patches the frame (throw_dropped)')
            self.s[BUDGET] = (self.s[BUDGET][0], self.s[BUDGET][1] - 1)
            if rqis or 'stale-rule' not in self.am:
                self.post_from(('Q', m, bid), frm)   # "goes back to its home as a requeue"
            else:
                self.note('requeue_if_stale false: dropped')
            if 'throw-resubscribe' in self.am:
                self.post_home(('U', g, p, cur, self.cfg.procs[p][1][pc] == 't'))
            return
        if fresh:
            if cur != seq:
                raise Violation('misaddressed', 'a deliver for %s#%d patches its wait #%d'
                                % (pname(self.cfg, p), seq, cur))
            self.receive(m, p)
            if bid:
                self.post_from(('T', bid, m), frm)   # m: the model's bookkeeping only
            return
        if (rqis or 'requeue-always' in self.mut) and 'no-requeue' not in self.mut:
            self.note('stale')
            self.post_from(('Q', m, bid), frm)
        else:
            self.note('stale: dropped (stale_dropped)')

    def drain(self, g, throw=False):
        q = self.s[ING][g]
        e = q[0]
        self.set_item(ING, g, q[1:])
        k = e[0]
        self.note('applies %s' % estr(self.cfg, e))
        if k == 'S':
            assert e[1] is None or g == self.homeg() or self.closed(H), (g, e)
            self.apply_send(g, e[1], e[2])
        elif k == 'U':
            self.apply_subscribe(g, *e[1:])
        elif k == 'X':
            self.apply_unsubscribe(g, *e[1:])
        elif k == 'D':
            self.apply_deliver(g, e, throw)
        elif k == 'Q':
            self.apply_requeue(g, e[1], e[2])
        elif k == 'T':
            self.apply_taken(g, e[1], e[2])
        elif k == 'P':
            self.apply_send(g, e[1], False)   # a host's post_message, applied as a local send
        elif k == 'A':
            self.apply_adopt(g, *e[1:])

    # --- a process's steps ---
    def step(self, p):
        pc, st, seq, exp = self.proc(p)
        g, ops = self.cfg.procs[p]
        op = ops[pc]
        if op == 'p':
            # a host thread's exec_pool::post_message: to the key's home (router.home_of)
            m = self.cfg.op_msg[(p, pc)]
            if self.post_home(('P', m)):
                self.mark_sent(g, m)
            else:
                self.set_msg(m, (M_REFUSED,) + self.msg(m)[1:])
            self.advance(p)
            return
        if op == 's':
            m = self.cfg.op_msg[(p, pc)]
            if self.local_home(g) == g:
                self.note('sends %s locally' % mname(m))
                self.mark_sent(g, m)
                self.advance(p)
                self.apply_send(g, m, False)
                return
            if self.post_home(('S', m, False)):
                self.mark_sent(g, m)
            else:
                self.set_msg(m, (M_REFUSED,) + self.msg(m)[1:])
            self.advance(p)
            return
        seq += 1
        timed = op == 't'
        if self.local_home(g) == g:
            loc, recs, pend, trs, nid = self.home(g)
            if pend:
                self.set_proc(p, (pc, WL, seq, exp))
                self.set_home(g, (loc, recs, pend[1:], trs, nid))
                self.note('receives locally: the front of the FIFO')
                self.assign(pend[0])
                self.receive(pend[0], p)
                return
            self.set_proc(p, (pc, WL, seq, exp))
            self.set_home(g, (loc + (p,), recs, pend, trs, nid))
            self.note('parks locally (#%d)' % seq)
            return
        self.set_proc(p, (pc, WR, seq, exp))
        self.note('parks (#%d)' % seq)
        self.post_home(('U', g, p, seq, timed))

    def end_wait(self, p, why):
        """A timeout or kill of a waiting process: leave the home's waiters, or unsubscribe."""
        pc, st, seq, exp = self.proc(p)
        g = self.cfg.procs[p][0]
        if st == WL:
            loc, recs, pend, trs, nid = self.home(g)
            self.set_home(g, (tuple(x for x in loc if x != p), recs, pend, trs, nid))
        elif st == WR and 'no-unsubscribe' not in self.mut:
            self.post_home(('X', g, p, seq))

    def timeout(self, p):
        self.note('%s times out' % pname(self.cfg, p))
        self.end_wait(p, 'timeout')
        self.advance(p)

    def kill(self, p):
        self.note('%s is killed' % pname(self.cfg, p))
        pc, st, seq, exp = self.proc(p)
        self.end_wait(p, 'kill')
        self.set_proc(p, (pc, DEAD, seq, exp))
        self.s[BUDGET] = (self.s[BUDGET][0] - 1, self.s[BUDGET][1])

    def expire(self, p):
        pc, st, seq, exp = self.proc(p)
        self.set_proc(p, (pc, st, seq, exp | (1 << seq)))
        self.note('%s#%d\'s deadline passes' % (pname(self.cfg, p), seq))

    # --- teardown ---
    def close_and_take(self, g):
        """Close g's handle and take what it holds, in the order to forward it."""
        self.set_item(CLOSED, g, True)
        q = self.s[ING][g]
        self.set_item(ING, g, ())
        queued = []
        for e in q:
            if e[0] == 'D':
                if e[6] or 'stale-rule' not in self.am:
                    queued.append(('to', e[1], ('Q', e[4], e[5])))   # "a deliver as a requeue"
            elif e[0] == 'S':
                queued.append(('home', ('S', e[1], e[2] or 'forward-uncapped' in self.am)))
            elif e[0] == 'P':
                if 'forward-host-posts' in self.am:
                    queued.append(('home', ('S', e[1], True)))
                else:
                    self.note('a host post of %s dies with %s (destroy_dropped)'
                              % (mname(e[1]), GROUPS[g]))
            else:
                queued.append(('home', e))
        recs = []
        pend = []
        adopts = []
        if g == H:
            loc, rs, pq, trs, nid = self.home(g)
            recs = [('home', ('U',) + r) for r in rs]
            pend = [('home', ('S', m, True)) for m in pq]
            for tr in trs:
                if 'adopt' in self.am:
                    adopts.append(('home', ('A', tr[1], tr[0], tr[2], tr[3])))
                else:
                    self.flag(tr[1], F_ORPHAN)
            self.set_home(g, EMPTY_HOME)
        elif self.home(g) != EMPTY_HOME:
            # an island's own FIFO and waiters (islands): its waiters are its processes, killed
            # next; it homes nothing for another group, so its own pending messages die with it,
            # counted (destroy_dropped), as a Stage 1 group's do (S2b.0a)
            for m in self.home(g)[2]:
                self.flag(m, F_DIED)
            if self.home(g)[2]:
                self.note("%s's own pending %s die with it (destroy_dropped)"
                          % (GROUPS[g], ', '.join(mname(m) for m in self.home(g)[2])))
            self.set_home(g, EMPTY_HOME)
        # an adopt goes before every queued entry, so a late answer finds its tracker
        if 'forward-pending-first' in self.am:
            buf = pend + recs + adopts + queued
        else:
            buf = adopts + queued + recs + pend
        self.note('%s closes its handle; to forward: %s' % (
            GROUPS[g], ', '.join(estr(self.cfg, x[2] if x[0] == 'to' else x[1]) for x in buf)
            or 'nothing'))
        return tuple(buf)

    def forward(self, item):
        if item[0] == 'to':
            self.post_from(item[2], item[1])
            return
        e = item[1]
        # the keys' new home, resolved without the group torn down: G0 (the only other home)
        ok = self.push(G0, e)
        self.note('forwards %s to G0%s' % (estr(self.cfg, e), '' if ok else
                                           ': refused (ingress full): lost'))

    def teardown(self, g):
        slot = TDH if g == H else TDR
        phase, buf = self.s[slot]
        rehome = 'rehome-order' in self.am
        if phase == TD_NONE and not rehome and g == H:
            self.s[REG] = False
            self.note('H\'s teardown: unregisters its prefix (the home is G0)')
            phase = TD_UNREG
        elif phase == TD_NONE or phase == TD_UNREG:
            buf = self.close_and_take(g)
            phase = TD_FWD if buf else (TD_FWDDONE if rehome and g == H else TD_KILL)
        elif phase == TD_FWD:
            self.forward(buf[0])
            buf = buf[1:]
            if not buf:
                phase = TD_FWDDONE if rehome and g == H else TD_KILL
        elif phase == TD_FWDDONE:
            self.s[REG] = False
            self.note('H unregisters its prefix (the home is G0)')
            phase = TD_KILL
        elif phase == TD_KILL:
            left = [p for p, (pg, _) in enumerate(self.cfg.procs)
                    if pg == g and self.proc(p)[1] in (RUN, WL, WR)]
            if left:
                p = left[0]
                pc, st, seq, exp = self.proc(p)
                self.note('%s\'s teardown kills %s' % (GROUPS[g], pname(self.cfg, p)))
                if st == WR and 'no-unsubscribe' not in self.mut:
                    self.post_home(('X', g, p, seq))
                self.set_proc(p, (pc, DEAD, seq, exp))
            else:
                self.note('%s\'s teardown: nothing left to kill' % GROUPS[g])
            if len(left) <= 1:
                phase = TD_DONE
        self.s[slot] = (phase, buf)


def estr(cfg, e):
    k = e[0]
    if k == 'S':
        if e[1] is None:
            return 'send(another key)'
        return 'send(%s%s)' % (mname(e[1]), ', uncapped' if e[2] else '')
    if k == 'U':
        return 'subscribe(%s#%d%s)' % (pname(cfg, e[2]), e[3], ', timed' if e[4] else '')
    if k == 'X':
        return 'unsubscribe(%s#%d)' % (pname(cfg, e[2]), e[3])
    if k == 'D':
        return 'deliver(%s#%d, %s from %s%s%s)' % (pname(cfg, e[2]), e[3], mname(e[4]),
                                                    GROUPS[e[1]],
                                                    ', ' + bname(e[5]) if e[5] else '',
                                                    '' if e[6] else ', not requeued if stale')
    if k == 'Q':
        return 'requeue(%s%s)' % (mname(e[1]), ', ' + bname(e[2]) if e[2] else '')
    if k == 'T':
        return 'taken(%s)' % bname(e[1])
    if k == 'P':
        return 'post(%s)' % mname(e[1])
    if k == 'A':
        return 'adopt(%s, %s, %d outstanding%s)' % (mname(e[1]), bname(e[2]), e[3],
                                                    ', taken' if e[4] else '')
    return str(e)


class Model(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.np = len(cfg.procs)
        self.nm = len(cfg.msg_proc)
        self.pgroup = [g for g, _ in cfg.procs]

    def initial(self):
        procs = tuple((0, RUN, 0, 0) for _ in self.cfg.procs)
        msgs = tuple((M_UNSENT, None, 0, 0, 0) for _ in range(self.nm))
        ing = ((('S', None, False),) if self.cfg.busy0 else (),) + ((),) * (NG - 1)
        return (True, (False,) * NG, ing, (EMPTY_HOME,) * NG, procs, msgs,
                ((),) * NG, (TD_NONE, ()), (TD_NONE, ()), (self.cfg.kills, self.cfg.throws))

    def busy(self, s, g):
        """g's thread is in its teardown, or done."""
        if g == H:
            return s[TDH][0] != TD_NONE
        if g == R1:
            return s[TDR][0] != TD_NONE
        return False

    def options(self, s):
        out = []
        kills, throws = s[BUDGET]
        for g in range(NG):
            if self.busy(s, g):
                continue
            q = s[ING][g]
            if q:
                out.append(('drain', g))
                e = q[0]
                if throws and e[0] == 'D' and s[PROCS][e[2]][1] == WR and \
                        s[PROCS][e[2]][2] == e[3]:
                    out.append(('throw', g))
        for p in range(self.np):
            g = self.pgroup[p]
            if self.busy(s, g):
                continue
            pc, st, seq, exp = s[PROCS][p]
            if st == RUN:
                out.append(('step', p))
            elif st in (WL, WR) and self.cfg.procs[p][1][pc] == 't':
                out.append(('timeout', p) if (exp >> seq) & 1 else ('expire', p))
            if kills and st in (RUN, WL, WR) and g != X:
                out.append(('kill', p))
        if self.cfg.teardown and s[TDH][0] != TD_DONE:
            out.append(('td', H))
        if self.cfg.rdie and s[TDR][0] != TD_DONE:
            out.append(('td', R1))
        return out

    def apply(self, s, label, oracle=(), notes=None):
        """One step; returns its Work (raises Violation, or Blocked when it is not enabled)."""
        w = Work(self, s, oracle, notes)
        kind, x = label
        if kind == 'drain':
            w.drain(x)
        elif kind == 'throw':
            w.drain(x, throw=True)
        elif kind == 'step':
            w.step(x)
        elif kind == 'timeout':
            w.timeout(x)
        elif kind == 'expire':
            w.expire(x)
        elif kind == 'kill':
            w.kill(x)
        elif kind == 'td':
            w.teardown(x)
        return w

    def successors(self, s, label):
        """Every outcome of one step over the internal choices it makes (choice=1): a list of
        (choices, state, soft violations); raises Violation or Blocked."""
        out = []
        stack = [()]
        while stack:
            pre = stack.pop()
            w = self.apply(s, label, pre)
            for j in range(len(pre), len(w.points)):
                for alt in range(1, w.points[j]):
                    stack.append(tuple(w.taken[:j]) + (alt,))
            out.append((tuple(w.taken), w.freeze(), w.soft))
        return out

    def actor(self, label):
        kind, x = label
        if kind in ('drain', 'throw', 'td'):
            return GROUPS[x]
        return GROUPS[self.pgroup[x]]

    def idle(self, s):
        if any(s[ING]):
            return False
        if s[TDH][0] not in (TD_NONE, TD_DONE) or s[TDR][0] not in (TD_NONE, TD_DONE):
            return False
        return not any(st == RUN for _, st, _, _ in s[PROCS])

    def idle_check(self, s):
        """The properties at an idle state; returns [(name, detail)]."""
        out = []
        homeg = H if s[REG] else G0
        on = router_on(self.cfg, s)
        pend_all = sum((hs[2] for hs in s[HOMES]), ())
        for m, (st, w, rin, rout, fl) in enumerate(s[MSGS]):
            if st != M_SENT:
                continue
            got = rin + rout
            inf = pend_all.count(m)
            if got == 0 and inf == 0 and not fl & F_DIED and (not fl & F_DROPPED or
                                                              fl & F_RDROP):
                out.append(('lost', '%s was sent and is neither received nor pending%s'
                            % (mname(m), ' (a rehomed message a full FIFO dropped)'
                               if fl & F_RDROP else '')))
            elif inf >= 2 or (inf and got):
                out.append(('duplicate', '%s is pending %d time(s) and was received %d time(s)%s'
                            % (mname(m), inf, got, ' (a broadcast its home dropped the tracking '
                               'of, with copies outstanding)' if fl & F_ORPHAN else '')))
        hs = s[HOMES][homeg]
        live_waits = []
        for p, (pc, st, seq, exp) in enumerate(s[PROCS]):
            if st in (WL, WR) and not (self.cfg.procs[p][1][pc] == 't' and (exp >> seq) & 1):
                live_waits.append(p)
        # where each live wait is served: a local waiter in its group, a remote one at the home
        where = {p: self.pgroup[p] if s[PROCS][p][1] == WL else homeg for p in live_waits}
        apart = self.cfg.islands and not s[REG]   # H gone: islands keep the key to themselves
        for h in range(NG):
            pend = s[HOMES][h][2]
            if not pend or not live_waits:
                continue
            here = [p for p in live_waits if where[p] == h]
            # an island's own message, sent once H was gone, beside its own local wait: its
            # in-group messaging, which must work whatever the router does
            own = [p for p in live_waits if s[PROCS][p][1] == WL and any(
                self.pgroup[self.cfg.msg_proc[m]] == self.pgroup[p] and s[MSGS][m][4] & F_LATE
                for m in pend)]
            ps = here or own or live_waits
            name = 'gap-island-rehomed' if apart and not here and not own else 'stranded'
            out.append((name, '%s pending at %s while %s waits (served at %s)'
                        % (', '.join(mname(m) for m in pend), GROUPS[h], pname(self.cfg, ps[0]),
                           GROUPS[where[ps[0]]])))
            if name == 'stranded':
                break
        for p in live_waits:
            pc, st, seq, exp = s[PROCS][p]
            g = self.pgroup[p]
            if st == WL:
                h = g
                ok = p in s[HOMES][g][0] and (
                    g == homeg or (self.cfg.islands and g in (R1, R2, S) and not s[REG]) or
                    not on)
            else:
                h = homeg
                ok = any(r[:3] == (g, p, seq) for r in hs[1])
            if not ok:
                out.append(('orphan-wait', '%s#%d waits and %s (the home%s) has no %s for it'
                            % (pname(self.cfg, p), seq, GROUPS[h],
                               '' if on else ': the router is off',
                               'local waiter' if st == WL else 'record')))
                break
        for h in range(NG):
            for r in s[HOMES][h][1]:
                pc, st, seq, exp = s[PROCS][r[1]]
                if not (st == WR and seq == r[2]):
                    out.append(('stale-record', '%s holds a record for %s#%d, which no longer '
                                'waits' % (GROUPS[h], pname(self.cfg, r[1]), r[2])))
                    break
            if s[HOMES][h][3]:
                t = s[HOMES][h][3][0]
                out.append(('tracker-leak', '%s still tracks %s (%s, %d outstanding)'
                            % (GROUPS[h], bname(t[0]), mname(t[1]), t[2])))
        return out

    def describe_state(self, s):
        parts = []
        if not s[REG]:
            parts.append('home G0')
        for g in range(NG):
            if s[CLOSED][g]:
                parts.append('%s closed' % GROUPS[g])
            if s[ING][g]:
                parts.append('%s<-[%s]' % (GROUPS[g], ', '.join(estr(self.cfg, e)
                                                                for e in s[ING][g])))
        if not router_on(self.cfg, s):
            parts.append('router off')
        for h in range(NG):
            loc, recs, pend, trs, nid = s[HOMES][h]
            bits = []
            if loc:
                bits.append('waiters ' + ' '.join(pname(self.cfg, p) for p in loc))
            if recs:
                bits.append('records ' + ' '.join('%s#%d%s' % (pname(self.cfg, r[1]), r[2],
                                                               '!' if r[3] and
                                                               (s[PROCS][r[1]][3] >> r[2]) & 1
                                                               else '') for r in recs))
            if pend:
                bits.append('fifo ' + ' '.join(mname(m) for m in pend))
            if trs:
                bits.append('tracks ' + ' '.join('%s(%s,%d%s)' % (bname(t[0]), mname(t[1]), t[2],
                                                                  ',taken' if t[3] else '')
                                                 for t in trs))
            if bits:
                parts.append('%s{%s}' % (GROUPS[h], '; '.join(bits)))
        for p, (pc, st, seq, exp) in enumerate(s[PROCS]):
            if st in (WL, WR):
                parts.append('%s %s#%d%s' % (pname(self.cfg, p), 'waits' if st == WL else
                                             'waits remotely', seq,
                                             ' (past its deadline)' if (exp >> seq) & 1 else ''))
            elif st == DEAD:
                parts.append('%s dead' % pname(self.cfg, p))
        return '; '.join(parts) or 'nothing'


def label_str(m, label):
    kind, x = label
    if kind in ('drain', 'throw', 'td'):
        return '%s %s' % (GROUPS[x], {'drain': 'drains', 'throw': 'drains (throws)',
                                      'td': 'teardown step'}[kind])
    return '%s %s' % (pname(m.cfg, x), kind)


def encode(s):
    """A state as bytes: pickled with no memo (Pickler.fast), so equal states give equal bytes
    whatever their tuples share, at about a third of marshal's size."""
    f = io.BytesIO()
    p = pickle.Pickler(f, 4)
    p.fast = True
    p.dump(s)
    return f.getvalue()


decode = pickle.loads


def peak_mb():
    """The process's peak memory in MB (best effort: 0 where it cannot be read)."""
    try:
        import resource
        kb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        return round(kb / (1024.0 * 1024.0 if sys.platform == 'darwin' else 1024.0))
    except ImportError:
        pass
    try:
        import ctypes
        from ctypes import wintypes

        class PMC(ctypes.Structure):
            _fields_ = [('cb', wintypes.DWORD), ('PageFaultCount', wintypes.DWORD)] + [
                (n, ctypes.c_size_t) for n in (
                    'PeakWorkingSetSize', 'WorkingSetSize', 'QuotaPeakPagedPoolUsage',
                    'QuotaPagedPoolUsage', 'QuotaPeakNonPagedPoolUsage',
                    'QuotaNonPagedPoolUsage', 'PagefileUsage', 'PeakPagefileUsage')]
        c = PMC()
        c.cb = ctypes.sizeof(c)
        k32 = ctypes.windll.kernel32
        k32.GetCurrentProcess.restype = wintypes.HANDLE
        info = ctypes.windll.psapi.GetProcessMemoryInfo
        info.argtypes = [wintypes.HANDLE, ctypes.POINTER(PMC), wintypes.DWORD]
        info(k32.GetCurrentProcess(), ctypes.byref(c), c.cb)
        return round(c.PeakWorkingSetSize / (1024.0 * 1024.0))
    except Exception:
        return 0


def explore(cfg, max_states=None, log=None, compact=True):
    """Every reachable state, breadth first. Returns a dict: states, transitions, idle_states,
    seconds, complete, peak_mb, violations {name: {detail, steps, trace}}. A state is kept as its
    encoding (encode; with `compact` false as itself, faster for small runs), with the index of
    its parent and the code of the step that reached it."""
    m = Model(cfg)
    s0 = m.initial()
    enc = encode if compact else (lambda s: s)
    dec = decode if compact else (lambda k: k)
    index = {enc(s0): 0}
    keys = [enc(s0)]
    parent = array.array('q', [-1])
    via = array.array('l', [0])
    acts = [None]           # code -> (label, choices)
    act_code = {}
    dq = collections.deque([0])
    viol = {}
    transitions = 0
    idle = 0
    t0 = time.time()

    def found(name, detail, i, act):
        if name not in viol:
            viol[name] = (detail, i, act)

    while dq:
        if max_states and len(keys) > max_states:
            break
        i = dq.popleft()
        s = dec(keys[i])
        enabled = 0
        if m.idle(s):
            idle += 1
            for name, detail in m.idle_check(s):
                found(name, detail, i, None)
        for label in m.options(s):
            try:
                outs = m.successors(s, label)
            except Blocked:
                continue
            except Violation as v:
                enabled += 1
                transitions += 1
                found(v.name, v.detail, i, (label, ()))
                continue
            enabled += 1
            if len({o[1] for o in outs}) > 1:
                found('nondeterministic', '%s: the next waiter the text leaves open changes the '
                      'outcome (%d outcomes)' % (label_str(m, label), len(outs)), i,
                      (label, outs[1][0]))
            for choices, s2, soft in outs:
                transitions += 1
                for name, detail in soft:
                    found(name, detail, i, (label, choices))
                k2 = enc(s2)
                if k2 not in index:
                    index[k2] = len(keys)
                    dq.append(len(keys))
                    keys.append(k2)
                    act = (label, choices)
                    code = act_code.get(act)
                    if code is None:
                        code = act_code[act] = len(acts)
                        acts.append(act)
                    parent.append(i)
                    via.append(code)
                    if log and len(keys) % 500000 == 0:
                        log('%d states, %d transitions, %.0fs' % (len(keys), transitions,
                                                                  time.time() - t0))
        if not enabled and not m.idle(s):
            found('deadlock', 'no step is enabled: ' + m.describe_state(s), i, None)
    res = {'config': cfg.describe(), 'states': len(keys), 'transitions': transitions,
           'idle_states': idle, 'seconds': round(time.time() - t0, 1), 'complete': not dq,
           'peak_mb': peak_mb(), 'violations': {}}
    for name, (detail, i, act) in sorted(viol.items()):
        path = []
        while parent[i] >= 0:
            path.append(acts[via[i]])
            i = parent[i]
        path.reverse()
        if act is not None:
            path.append(act)
        trace = render(m, path, name)
        res['violations'][name] = {'detail': detail, 'steps': len(path), 'trace': trace}
    return res


def render(m, path, name=None):
    """Replay `path` from the initial state, one line per step: the actor, what it did, and the
    state after it; the last line names the violation."""
    s = m.initial()
    out = ['initial: ' + m.describe_state(s)]
    for label, choices in path:
        notes = []
        try:
            w = m.apply(s, label, choices, notes)
        except Violation as v:
            out.append('%s: %s  -> VIOLATION %s: %s' % (label_str(m, label), '; '.join(notes),
                                                       v.name, v.detail))
            return out
        s = w.freeze()
        line = '%s: %s  [%s]' % (label_str(m, label), '; '.join(notes) or label[0],
                                 m.describe_state(s))
        for vn, detail in w.soft:
            if vn == name:
                line += '  -> %s: %s' % (vn, detail)
        out.append(line)
    if m.idle(s):
        for vn, detail in m.idle_check(s):
            if name is None or vn == name:
                out.append('idle -> VIOLATION %s: %s' % (vn, detail))
                break
    elif name == 'deadlock':
        out.append('-> DEADLOCK: no step is enabled')
    if name == 'nondeterministic':
        out.append('-> VIOLATION nondeterministic: the last step, with the other choice of the '
                   'next waiter, gives another state')
    return out


# ----------------------------------------------------------------------------------------------
# configurations and the CI suites

def parse_procs(text):
    out = []
    for part in text.split(','):
        part = part.strip()
        if part:
            g, _, ops = part.partition(':')
            out.append((g.strip(), ops.strip()))
    return out


def parse_kv(text):
    out = {}
    if text:
        for part in text.split(','):
            k, _, v = part.partition('=')
            out[k.strip()] = int(v)
    return out


def make_config(procs, opts=None, amend=(), mutation=()):
    o = dict(kills=0, teardown=1, rdie=0, throws=0, icap=0, fcap=0, fcap0=0, choice=0, busy0=0,
             islands=0)
    o.update(opts or {})
    if isinstance(procs, str):
        procs = parse_procs(procs)
    return Config(procs, amend=amend, mutation=mutation, **o)


ALL = 'all'
BUT = lambda *names: tuple(a for a in AMENDMENTS if a not in names)  # noqa: E731
NO_TD = dict(teardown=0)
# Each check: (name, procs, opts, amend, mutation, must, allowed): the violations found must
# include every name in `must` and be among `must`, `allowed` and the gaps (always allowed). The
# frozen protocol's findings are each isolated by every amendment but the one that closes it, on
# the smallest configuration that shows it.
QUICK = [
    # every amendment: nothing found but the documented gaps
    ('amended', 'S:ss,R1:t,R2:t', {}, ALL, (), (), ()),
    ('amended-kills', 'S:s,G0:t,R1:t', dict(kills=1), ALL, (), (), ()),
    ('amended-receiver-teardown', 'S:s,H:r,R1:t,R2:r', dict(rdie=1, throws=1, choice=1), ALL, (),
     (), ()),
    ('amended-caps', 'S:ss,R1:t,R2:r', dict(icap=1, fcap=1, fcap0=1, busy0=1), ALL, (), (), ()),
    ('amended-islands', 'S:ss,R1:t,R2:r', dict(islands=1, kills=1), ALL, (), (), ()),
    ('amended-island-teardown', 'R1:r,R1:s,R2:t', dict(islands=1, rdie=1), ALL, (), (), ()),
    ('amended-host-posts', 'X:p,S:s,R1:t,R2:r', dict(icap=1), ALL, (), (), ()),
    # the documented gaps, which the amended protocol keeps
    ('gap-island-rehomed', 'S:s,R1:r', dict(islands=1), ALL, (), ('gap-island-rehomed',), ()),
    ('gap-requeue-order', 'S:ss,R1:tr,R2:r', NO_TD, ALL, (), ('gap-requeue-order',), ()),
    # each finding of the frozen text (README.md), without its amendment
    ('finding-rehome-order', 'S:ss,R1:t', {}, BUT('rehome-order'), (), ('fifo', 'stale-record'),
     ()),
    # adopt needs rehome-order's wait: a late answer that reaches G0 before the adopt is applied
    # untracked, and the tracker then waits for it for ever
    ('finding-rehome-order-adopt', 'H:s,R1:t,R2:t', {}, BUT('rehome-order'), (),
     ('duplicate', 'tracker-leak'), ('stale-record',)),
    ('finding-pending-first', 'S:ss,R1:r,R2:r', {}, BUT('forward-pending-first'), (),
     ('fifo', 'duplicate'), ()),
    ('finding-forward-uncapped', 'S:ss,R1:r', dict(icap=1, busy0=1), BUT('forward-uncapped'), (),
     ('lost',), ('fifo',)),
    ('finding-uncapped-past-cap', 'S:ss,R1:r', dict(fcap0=1), BUT('uncapped-past-cap'), (),
     ('lost',), ()),
    ('finding-stale-rule-teardown', 'S:s,H:r,R1:r', dict(NO_TD, rdie=1), BUT('stale-rule'), (),
     ('duplicate',), ()),
    ('finding-stale-rule-throw', 'S:s,H:r,R1:r,R2:r', dict(NO_TD, throws=1), BUT('stale-rule'), (),
     ('duplicate',), ()),
    ('finding-throw-resubscribe', 'S:s,R1:r', dict(NO_TD, throws=1), BUT('throw-resubscribe'), (),
     ('orphan-wait', 'stranded'), ()),
    ('finding-refused-stale', 'S:s,R1:r', dict(NO_TD, rdie=1), BUT('refused-stale'), (),
     ('lost',), ()),
    ('finding-queue-as-requeue', 'S:s,H:r,R1:t,R2:t', NO_TD, BUT('queue-as-requeue'), (),
     ('stranded',), ()),
    # per-home ids collide at the new home (without adopt: an orphan answer ends its tracker;
    # with it, deep-finding-unique-ids-adopt: two trackers under one id, and a message is lost)
    ('finding-unique-ids', 'S:ss,R1:rr,R2:r,R2:t', {}, BUT('unique-ids'), (),
     ('id-collision',), ()),
    ('finding-next-waiter-order', 'S:s,H:r,R1:t,R2:r', dict(NO_TD, choice=1),
     BUT('next-waiter-order'), (), ('nondeterministic',), ()),
    ('finding-island-rule', 'R1:r,R1:s,R2:t', dict(islands=1), BUT('island-rule'), (),
     ('stranded',), ()),
    ('finding-adopt', 'H:s,R1:t,R2:t', {}, BUT('adopt'), (), ('duplicate',), ()),
    ('finding-forward-host-posts', 'X:pp,R1:r', {}, BUT('forward-host-posts'), (), ('lost',),
     ('fifo',)),
    # the alternative to rehome-order and forward-pending-first: no duplicate, the reorder kept
    # the mutations, on the amended protocol
    ('mutation-no-requeue', 'S:s,R1:t', NO_TD, ALL, ('no-requeue',), ('lost',), ()),
    ('mutation-requeue-always', 'S:s,H:r,R1:t', NO_TD, ALL, ('requeue-always',), ('duplicate',),
     ()),
    ('mutation-untracked', 'S:s,R1:t,R2:r', NO_TD, ALL, ('untracked',), ('duplicate',), ()),
    ('mutation-no-unsubscribe', 'R1:t', NO_TD, ALL, ('no-unsubscribe',), ('stale-record',), ()),
    ('mutation-subscribe-no-fifo', 'S:s,R1:r', NO_TD, ALL, ('subscribe-no-fifo',), ('stranded',),
     ()),
    ('mutation-deliver-no-seq', 'S:s,R1:tr', NO_TD, ALL, ('deliver-no-seq',), ('misaddressed',),
     ()),
]
DEEP = [
    # the frozen text, every finding but unique-ids' at once: nothing else found
    ('deep-frozen', 'S:ss,R1:t,R2:r,H:r', dict(rdie=1, throws=1, icap=1, fcap0=1, busy0=1), (), (),
     ('fifo', 'stale-record', 'duplicate', 'lost', 'orphan-wait', 'stranded'), ('tracker-leak',)),
    ('deep-frozen-choice-islands', 'S:s,H:r,R1:t,R2:r', dict(choice=1, islands=1, kills=1), (),
     (), ('nondeterministic', 'stranded', 'duplicate', 'stale-record'), ('orphan-wait',)),
    # every amendment at larger bounds: nothing but the gaps
    ('deep-amended', 'S:ss,R1:t,R2:t,H:t', dict(kills=1), ALL, (), (), ()),
    ('deep-amended-g0', 'S:ss,R1:t,R2:t,G0:t', {}, ALL, (), (), ()),
    ('deep-amended-receiver-teardown', 'S:ss,R1:t,R2:t', dict(rdie=1, throws=1), ALL, (), (), ()),
    ('deep-amended-choice', 'H:t,S:ss,R1:t,R2:t', dict(choice=1), ALL, (), (), ()),
    ('deep-amended-caps', 'S:ss,R1:tr,R2:t', dict(icap=1, fcap=1, fcap0=1, busy0=1), ALL, (), (),
     ()),
    ('deep-amended-two-senders', 'S:ss,R1:st,R2:t', {}, ALL, (), (), ()),
    ('deep-amended-ids', 'S:ss,R1:rr,R2:r,R2:t', dict(kills=1), ALL, (), (), ()),
    ('deep-amended-islands', 'S:ss,R1:t,R2:t', dict(islands=1, rdie=1, kills=1), ALL, (), (), ()),
    ('deep-amended-host-posts', 'X:pp,R1:t,R2:t', dict(kills=1, icap=1), ALL, (), (), ()),
    # the collision's effect: a message lost (the invariant stops before it in the quick check)
    ('deep-finding-unique-ids-lost', 'H:s,S:s,R1:tr,R2:tr', {}, BUT('unique-ids'), (),
     ('lost', 'id-collision'), ()),
]


def say(msg):
    print(msg, flush=True)


def run_check(check, out=say, compact=True):
    """Run one suite check; True when it holds."""
    name, procs, opts, amend, mutation, must, allowed = check
    r = explore(make_config(procs, opts, amend, mutation), compact=compact)
    got = set(r['violations'])
    ok = r['complete'] and set(must) <= got <= set(must) | set(allowed) | set(GAPS)
    out('%-30s %s %9d states %10d transitions %7.1fs %6d MB  found %s%s'
        % (name, 'ok  ' if ok else 'FAIL', r['states'], r['transitions'], r['seconds'],
           r['peak_mb'],
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
    bad = sum(not run_check(c, out, compact=c not in QUICK) for c in checks)
    out('router.py: %d checks, %d unexpected, %.0fs' % (len(checks), bad, time.time() - t0))
    return bad


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--procs', default='S:ss,R1:t,R2:r')
    ap.add_argument('--opts', default='')
    ap.add_argument('--amend', default='all')
    ap.add_argument('--but', default='', help='every amendment but these (overrides --amend)')
    ap.add_argument('--mutation', default='')
    ap.add_argument('--max-states', type=int, default=0)
    ap.add_argument('--suite', choices=('quick', 'deep'))
    args = ap.parse_args(argv)
    if args.suite:
        checks = QUICK if args.suite == 'quick' else QUICK + DEEP
        return 1 if run_suite(checks) else 0
    amend = ALL if args.amend == 'all' else tuple(
        x for x in args.amend.split(',') if x and x != 'none')
    if args.but:
        amend = BUT(*[x for x in args.but.split(',') if x])
    mutation = tuple(x for x in args.mutation.split(',') if x)
    cfg = make_config(args.procs, parse_kv(args.opts), amend, mutation)
    r = explore(cfg, args.max_states or None,
                log=lambda msg: print(msg, file=sys.stderr, flush=True))
    print(json.dumps(r, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
