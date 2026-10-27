"""Executable model of docs/STATE_EXEC_LOCKS_PROTOCOL.md (its round-5 revision of 2026-10-27).

Revision 5b of the model (round-5 spec; README.md names it): lock-query's generation is the
largest gen among the X holds with the key (4.13, mutation 'query-first-gen'), checked by the world
at quiescence; L4's group teardown (5.7) is a world configuration.

Revision 5 of the model (round-4 spec; README.md names it): a home's entry carries its progress
cell (2.1), attach_home takes it and makes the home's empty key-cache entry (4.4), ref requires an
attached home and unref keeps the entry (4.15), detach_home reads no port and deletes the entry
(4.11; the closed assertion is the world's destroy action's), blocking_sinks returns (Pid, time)
pairs (4.13), query is implemented (4.13), and check() asserts an entry exactly for each attached
home (I9).

Revision 4 of the model (round-3 spec): a port post is one critical section under the port's own
mutex (2.3), and ~lock_home closes the port under it before calling detach_home, which no longer
writes `closed` (4.11, 5.7); the split-post configuration is expected to pass and its mutation
'post-test-outside-mu' to fail. Key-cache references are manager state (M.cache_refs, 2.4, 4.15),
taken and dropped under M.mu by try_acquire/block on a miss or an eviction and dropped for good by
detach_home (mutation 'no-unref-on-detach'). A DeadlockError cycle shows a pid only for a process
of the requester's home and owner (4.6).

Revision 3 of the model (spec of 07:25): 3.3's inheritance is the transitive definition only
(checked against an independent path enumeration, never a direct-set recursion); 4.3's
implementation-note seeds are the changed/granted/ended processes, not finish's `seeds`;
`blocking_sinks` (4.13) is implemented and checked at quiescence; owner checks (I12) are
exercised by foreign release/cancel calls; `attach_home` may run after processes have locks
(the false->true lending case); `M.options.inheritance` false is a configuration; and a port
post can be split into its closed test and its append/signal (section 2.3 read literally).

Revision 2 of the model: `applies` and `lends` are total (2.1); `relevel_all` tests `skip` before
`applies`; 4.3's implementation note is checked in all four lending cases, as a hard violation;
the trie keeps an all-zero ancestor of an abandoned flag (2.4); `relevel` coalescing (4.3) is an
option; every manager operation is followed by Manager.check(); inheritance is checked against an
independent path enumeration (3.3, cap 8).

The manager (section 4) is implemented literally, one Python method per operation. Each method is
one atomic step of the model (the manager mutex). Homes (section 5) are modelled with a list of
pending micro-operations, so that an operation, each delivery of its notifications to another
home, and the private code after them are separate atomic steps between which other homes act.

Ghost checks (the invariants of section 6 and a few properties the task asks for) are evaluated
inside the operations and after every step by World.check().

stdlib only.
"""

import collections

INF = 2 ** 31 - 1
S, X = 'S', 'X'
STATS = collections.Counter()   # coverage counters, not part of the state


class Violation(Exception):
    def __init__(self, name, detail):
        super().__init__(name + ': ' + detail)
        self.name = name
        self.detail = detail


# ---------------------------------------------------------------------------------------------
# 1. Keys, requests, conflict

def prefix(a, b):
    return len(a) <= len(b) and b[:len(a)] == a


def overlap(a, b):
    return prefix(a, b) or prefix(b, a)


def compat(m, n):
    return m == S and n == S


def conflicts(ak, am, bk, bm):
    if am == S and bm == S:
        return False
    for a in ak:
        for b in bk:
            if overlap(a, b):
                return True
    return False


def under(hkeys, k):
    for hk in hkeys:
        if prefix(hk, k):
            return True
    return False


def canonical(keys):
    out = []
    for k in sorted(set(keys)):
        if any(prefix(o, k) for o in out):
            continue
        out.append(k)
    return tuple(out)


def K(s):
    return tuple(s.split('.'))


def kstr(k):
    return '.'.join(k)


def req_str(keys, mode):
    return '%s{%s}' % (mode, ','.join(kstr(k) for k in keys))


# ---------------------------------------------------------------------------------------------
# 2. State

class Rec(object):
    __slots__ = ('owner', 'base', 'holds', 'ticket', 'inh', 'rl')

    def __init__(self, owner, base):
        self.owner = owner
        self.base = base
        self.holds = set()
        self.ticket = None
        self.inh = INF
        self.rl = False          # 4.3 coalescing: a relevel for this process is in a port

    def clone(self):
        r = Rec(self.owner, self.base)
        r.holds = set(self.holds)
        r.ticket = self.ticket
        r.inh = self.inh
        r.rl = self.rl
        return r

    def key(self):
        return (self.owner, self.base, tuple(sorted(self.holds)), self.ticket, self.inh, self.rl)


class Hold(object):
    __slots__ = ('pid', 'keys', 'mode', 'gen', 'body_started', 'taken')

    def __init__(self, pid, keys, mode, gen, body_started, taken):
        self.pid = pid
        self.keys = keys
        self.mode = mode
        self.gen = gen
        self.body_started = body_started
        self.taken = taken

    def clone(self):
        return Hold(self.pid, self.keys, self.mode, self.gen, self.body_started, self.taken)

    def key(self):
        return (self.pid, self.keys, self.mode, self.gen, self.body_started, tuple(sorted(self.taken)))


class Ticket(object):
    __slots__ = ('pid', 'keys', 'mode', 'seq', 'state', 'hold', 'error')

    def __init__(self, pid, keys, mode, seq, state, hold=None, error=None):
        self.pid = pid
        self.keys = keys
        self.mode = mode
        self.seq = seq
        self.state = state
        self.hold = hold
        self.error = error

    def clone(self):
        return Ticket(self.pid, self.keys, self.mode, self.seq, self.state, self.hold, self.error)

    def key(self):
        return (self.pid, self.keys, self.mode, self.seq, self.state, self.hold, self.error)


class Port(object):
    __slots__ = ('queue', 'closed')

    def __init__(self):
        self.queue = []
        self.closed = False

    def clone(self):
        p = Port()
        p.queue = list(self.queue)
        p.closed = self.closed
        return p


class Cell(object):
    """2.1's `progress`: a shared_ptr<atomic time> shared by a home and its M.homes entry (rev 5).
    Time is not modelled: the world stores a logical clock in it; no invariant reads the value."""
    __slots__ = ('value',)

    def __init__(self, value=0):
        self.value = value

    def load(self):
        return self.value


class Node(object):
    __slots__ = ('here', 'sub', 'wq', 'subw', 'ab', 'children', 'refs')

    def __init__(self):
        self.here = {S: set(), X: set()}
        self.sub = {S: 0, X: 0}
        self.wq = set()
        self.subw = 0
        self.ab = False
        self.children = set()
        self.refs = 0           # rev 4: key-cache references (2.4, M.cache_refs)

    def clone(self):
        n = Node()
        n.here = {S: set(self.here[S]), X: set(self.here[X])}
        n.sub = dict(self.sub)
        n.wq = set(self.wq)
        n.subw = self.subw
        n.ab = self.ab
        n.children = set(self.children)
        n.refs = self.refs
        return n

    def empty(self):
        return (not self.here[S] and not self.here[X] and self.sub[S] == 0 and self.sub[X] == 0
                and not self.wq and self.subw == 0 and not self.ab and self.refs == 0)


# ---------------------------------------------------------------------------------------------
# 4. The manager

class Manager(object):
    """Literal implementation of section 4. Every public method is one `with M:` block."""

    def __init__(self, inheritance=True, mutation=None, coalesce=False, one_home=False):
        self.inheritance = inheritance
        self.coalesce = coalesce
        self.one_home = one_home            # rev 5 (4.4): before L3 a manager accepts one home
        self.mutation = mutation or set()   # spec mutations, for checking the checker
        self.homes = {}          # HomeId -> [prio, Port, progress Cell] (2.1)
        self.recs = {}
        self.holds = {}
        self.tickets = {}
        self.queue = set()
        self.abandoned = set()
        self.next_seq = 0
        self.next_hold = 0
        self.next_gen = 0
        self.trie = {(): Node()}
        self.cache_refs = {}     # rev 4: HomeId -> {Key: count} (2.4, 4.15)
        # ghosts
        self.g_pending_report = set()   # abandonment reports not yet seen by a started X body
        self.g_last_x_gen = 0
        self.g_max_gen = 0
        self.soft = []                  # (name, detail) of soft violations found in this step
        self.N = None
        self._start_inh = None
        self._start_graph = None
        self._seeds = set()
        self._lending_before = False

    def clone(self, ports, cells):
        m = Manager.__new__(Manager)
        m.inheritance = self.inheritance
        m.coalesce = self.coalesce
        m.one_home = self.one_home
        m.mutation = self.mutation
        m.homes = dict((h, [v[0], ports[h], cells[h]]) for h, v in self.homes.items())
        m.recs = dict((p, r.clone()) for p, r in self.recs.items())
        m.holds = dict((i, h.clone()) for i, h in self.holds.items())
        m.tickets = dict((i, t.clone()) for i, t in self.tickets.items())
        m.queue = set(self.queue)
        m.abandoned = set(self.abandoned)
        m.next_seq = self.next_seq
        m.next_hold = self.next_hold
        m.next_gen = self.next_gen
        m.trie = dict((k, n.clone()) for k, n in self.trie.items())
        m.cache_refs = dict((h, dict(d)) for h, d in self.cache_refs.items())
        m.g_pending_report = set(self.g_pending_report)
        m.g_last_x_gen = self.g_last_x_gen
        m.g_max_gen = self.g_max_gen
        m.soft = []
        m.N = None
        m._start_inh = None
        m._start_graph = None
        m._seeds = set()
        m._lending_before = False
        return m

    def key(self):
        return self.lock_key() + (tuple(sorted((h, tuple(sorted(d.items())))
                                               for h, d in self.cache_refs.items())),)

    def lock_key(self):
        # every manager field but the key-cache references (I12, I13 compare this)
        return (tuple(sorted((p, r.key()) for p, r in self.recs.items())),
                tuple(sorted((i, h.key()) for i, h in self.holds.items())),
                tuple(sorted((i, t.key()) for i, t in self.tickets.items())),
                tuple(sorted(self.queue)), tuple(sorted(self.abandoned)),
                self.next_seq, self.next_hold, self.next_gen,
                tuple(sorted((h, v[0]) for h, v in self.homes.items())),
                tuple(sorted(self.g_pending_report)), self.g_last_x_gen)

    # -- misc ---------------------------------------------------------------------------------
    def fail(self, name, detail):
        raise Violation(name, detail)

    def soft_fail(self, name, detail):
        self.soft.append((name, detail))

    def require(self, cond, what):
        if not cond:
            self.fail('require', what)

    def applies(self, P):
        # 2.1: total; short-circuit, so a deleted home is never read
        h = self.homes.get(P[0])
        return h is not None and h[0]

    def lends(self, P):
        return self.inheritance and self.applies(P)

    def lending(self):
        return any(self.inheritance and v[0] for v in self.homes.values())

    # -- 3.1 blocking ---------------------------------------------------------------------------
    def blk_hold(self, P, keys, mode):
        return [h for h in self.holds.values() if h.pid != P and conflicts(keys, mode, h.keys, h.mode)]

    def exempt(self, P, U):
        if 'no-exempt' in self.mutation:
            return False
        for h in self.holds.values():
            if h.pid == P and conflicts(U.keys, U.mode, h.keys, h.mode):
                return True
        return False

    def blk_wait(self, P, keys, mode, s):
        out = []
        for t in sorted(self.queue):
            U = self.tickets[t]
            if U.seq < s and U.pid != P and conflicts(keys, mode, U.keys, U.mode) and not self.exempt(P, U):
                out.append(U)
        return out

    def grantable(self, P, keys, mode, s):
        bh = self.blk_hold(P, keys, mode)
        bw = self.blk_wait(P, keys, mode, s)
        self._trie_cross_check(P, keys, mode, s, bh, bw)
        return not bh and not bw

    def grantable_t(self, T):
        return self.grantable(T.pid, T.keys, T.mode, T.seq)

    # -- 3.2 graph ------------------------------------------------------------------------------
    def succ_of(self, P):
        rec = self.recs.get(P)
        if rec is None or rec.ticket is None:
            return set()
        T = self.tickets[rec.ticket]
        if T.state != 'queued':
            return set()
        return (set(h.pid for h in self.blk_hold(P, T.keys, T.mode))
                | set(U.pid for U in self.blk_wait(P, T.keys, T.mode, T.seq)))

    def graph(self):
        g = {}
        for P, r in self.recs.items():
            if r.ticket is not None and self.tickets[r.ticket].state == 'queued':
                g[P] = self.succ_of(P)
        return g

    @staticmethod
    def reach_in(g, S0):
        seen = set(S0)
        stack = list(S0)
        while stack:
            a = stack.pop()
            for b in g.get(a, ()):
                if b not in seen:
                    seen.add(b)
                    stack.append(b)
        return seen

    @staticmethod
    def reach_depth(g, S0, depth):
        seen = set(S0)
        frontier = list(S0)
        for _ in range(depth):
            nxt = []
            for a in frontier:
                for b in g.get(a, ()):
                    if b not in seen:
                        seen.add(b)
                        nxt.append(b)
            frontier = nxt
        return seen

    @staticmethod
    def find_cycle(g):
        WHITE, GREY, BLACK = 0, 1, 2
        col = {}
        for s in sorted(g):
            if col.get(s, WHITE) != WHITE:
                continue
            stack = [(s, iter(sorted(g.get(s, ()))))]
            path = [s]
            col[s] = GREY
            while stack:
                a, it = stack[-1]
                nxt = next(it, None)
                if nxt is None:
                    col[a] = BLACK
                    stack.pop()
                    path.pop()
                    continue
                c = col.get(nxt, WHITE)
                if c == GREY:
                    return path[path.index(nxt):] + [nxt]
                if c == WHITE:
                    col[nxt] = GREY
                    path.append(nxt)
                    stack.append((nxt, iter(sorted(g.get(nxt, ())))))
        return None

    # -- 3.3 inheritance ------------------------------------------------------------------------
    def anc8(self, Xp, pred):
        seen = set()
        frontier = [Xp]
        for _ in range(8):
            nxt = []
            for a in frontier:
                for y in pred.get(a, ()):
                    if y not in seen and y != Xp:
                        seen.add(y)
                        nxt.append(y)
            frontier = nxt
        return seen

    def inh_formula(self, Xp, pred, lending):
        if not lending:
            return INF
        best = INF
        for Y in self.anc8(Xp, pred):
            if self.lends(Y):
                b = self.recs[Y].base
                if b < best:
                    best = b
        return best

    @staticmethod
    def pred_of(g):
        pred = {}
        for a, bs in g.items():
            for b in bs:
                pred.setdefault(b, set()).add(a)
        return pred

    # -- trie (2.4) -----------------------------------------------------------------------------
    def _node(self, k):
        n = self.trie.get(k)
        if n is not None:
            return n
        parent = self._node(k[:-1])
        n = Node()
        self.trie[k] = n
        parent.children.add(k)
        return n

    def _maybe_free(self, k):
        # 2.4: a node is freed when all counts are zero, lists empty, flag clear (and, so the
        # tree stays connected, it has no child)
        while k != ():
            n = self.trie.get(k)
            if n is None or not n.empty() or n.children:
                return
            del self.trie[k]
            self.trie[k[:-1]].children.discard(k)
            k = k[:-1]

    def _trie_add_hold(self, hid, h):
        for k in h.keys:
            self._node(k).here[h.mode].add(hid)
            for i in range(len(k) + 1):
                self.trie[k[:i]].sub[h.mode] += 1

    def _trie_remove_hold(self, hid, h):
        for k in h.keys:
            self.trie[k].here[h.mode].discard(hid)
            for i in range(len(k) + 1):
                self.trie[k[:i]].sub[h.mode] -= 1
        for k in h.keys:
            self._maybe_free(k)

    def _trie_enqueue(self, T):
        for k in T.keys:
            self._node(k).wq.add(T.seq)
            for i in range(len(k) + 1):
                self.trie[k[:i]].subw += 1

    def _trie_dequeue(self, T):
        for k in T.keys:
            self.trie[k].wq.discard(T.seq)
            for i in range(len(k) + 1):
                self.trie[k[:i]].subw -= 1
        for k in T.keys:
            self._maybe_free(k)

    def _abandon_add(self, keys):
        for k in keys:
            self.abandoned.add(k)
            self._node(k).ab = True

    def _abandon_remove(self, keys):
        for k in keys:
            self.abandoned.discard(k)
            n = self.trie.get(k)
            if n is not None:
                n.ab = False
                self._maybe_free(k)

    def _trie_enum_holds(self, keys, mode):
        cm = (X,) if mode == S else (S, X)
        res = set()
        for k in keys:
            for i in range(1, len(k) + 1):
                n = self.trie.get(k[:i])
                if n is None:
                    break
                for m in cm:
                    res |= n.here[m]
            n = self.trie.get(k)
            if n is None:
                continue
            stack = list(n.children)
            while stack:
                c = stack.pop()
                cn = self.trie[c]
                if any(cn.sub[m] for m in cm):
                    for m in cm:
                        res |= cn.here[m]
                    stack.extend(cn.children)
        return res

    def _trie_enum_waiters(self, keys):
        res = set()
        for k in keys:
            for i in range(1, len(k) + 1):
                n = self.trie.get(k[:i])
                if n is None:
                    break
                res |= n.wq
            n = self.trie.get(k)
            if n is None:
                continue
            stack = list(n.children)
            while stack:
                c = stack.pop()
                cn = self.trie[c]
                if cn.subw > 0:
                    res |= cn.wq
                    stack.extend(cn.children)
        return res

    def _trie_cross_check(self, P, keys, mode, s, bh, bw):
        hs = set(i for i in self._trie_enum_holds(keys, mode) if self.holds[i].pid != P
                 and conflicts(keys, mode, self.holds[i].keys, self.holds[i].mode))
        ref = set(i for i, h in self.holds.items() if any(h is b for b in bh))
        if hs != ref:
            self.fail('I4-enum', 'trie hold enumeration %r != definition %r for %s' % (hs, ref, req_str(keys, mode)))
        ws = set()
        for t in self._trie_enum_waiters(keys):
            U = self.tickets[t]
            if U.seq < s and U.pid != P and conflicts(keys, mode, U.keys, U.mode) and not self.exempt(P, U):
                ws.add(t)
        if ws != set(U.seq for U in bw):
            self.fail('I4-enum', 'trie waiter enumeration differs for %s' % req_str(keys, mode))

    # -- 4.15 key-cache references -------------------------------------------------------------
    def _ref(self, H, k):
        # 4.15: require H in M.homes; M.cache_refs[H] += { k }; the node of k (made if absent)
        # gets refs += 1. Rev 5: the entry exists from attach_home on (I9), so no setdefault.
        self.require(H in self.homes and H in self.cache_refs, 'ref: H%s is not attached' % H)
        d = self.cache_refs[H]
        d[k] = d.get(k, 0) + 1
        self._node(k).refs += 1

    def _unref(self, H, k):
        # 4.15: require k in M.cache_refs[H]; one occurrence goes; the entry stays, maybe empty,
        # until detach_home deletes it (rev 5); the node of k gets refs -= 1 and may be freed
        d = self.cache_refs.get(H)
        self.require(d is not None and d.get(k, 0) > 0, 'unref: no reference of H%s on %s' % (H, kstr(k)))
        d[k] -= 1
        if d[k] == 0:
            del d[k]                # the multiset loses its last occurrence of k
        if not d and 'unref-pops-entry' in self.mutation:
            del self.cache_refs[H]  # the rev-4 behaviour: I9 must catch it
        self.trie[k].refs -= 1
        self._maybe_free(k)

    def _apply_refs(self, H, refs):
        add, drop = refs
        for k in add:           # a miss: the new entry's reference
            self._ref(H, k)
        for k in drop:          # an eviction
            self._unref(H, k)

    def ref_nodes(self, H, keys):
        self._begin()
        for k in keys:
            self._ref(H, k)
        return []

    def unref_nodes(self, H, keys):
        self._begin()
        for k in keys:
            self._unref(H, k)
        return []

    # -- 4.1 helpers ------------------------------------------------------------------------------
    def ensure_rec(self, P, info):
        if P not in self.recs:
            self.recs[P] = Rec(info[0], info[1])

    def new_hold(self, P, keys, mode, started, s):
        # ghost I1 and I5 at the moment of the grant
        if self.blk_hold(P, keys, mode):
            self.fail('I1', 'hold created for %r %s while another process holds a conflicting hold'
                      % (P, req_str(keys, mode)))
        for t in sorted(self.queue):
            U = self.tickets[t]
            if U.seq < s and U.pid != P and conflicts(keys, mode, U.keys, U.mode) and self.exempt(P, U):
                STATS['grant_by_conversion_rule'] += 1
            if U.seq < s and U.pid != P and conflicts(keys, mode, U.keys, U.mode) and not self.exempt(P, U):
                self.fail('I5', 'hold for %r %s (seq %s) granted ahead of earlier conflicting waiter %r seq %d'
                          % (P, req_str(keys, mode), s, U.pid, U.seq))
        self.next_hold += 1
        hid = self.next_hold
        if mode == X:
            self.next_gen += 1
            gen = self.next_gen
            taken = frozenset(k for k in keys if k in self.abandoned)
            self._abandon_remove(taken)
        else:
            gen = self.next_gen
            taken = frozenset()
        # ghost I15
        if mode == X:
            if gen <= self.g_max_gen:
                self.fail('I15', 'X gen %d not greater than earlier %d' % (gen, self.g_max_gen))
            self.g_last_x_gen = gen
        else:
            if gen != self.g_last_x_gen:
                self.fail('I15', 'S gen %d != last X gen %d' % (gen, self.g_last_x_gen))
        self.g_max_gen = max(self.g_max_gen, gen)
        h = Hold(P, keys, mode, gen, started, taken)
        self.holds[hid] = h
        self._trie_add_hold(hid, h)
        self.recs[P].holds.add(hid)
        if started:
            self._g_body_start(h)
        return hid

    def _g_body_start(self, h):
        if h.taken:
            STATS['abandoned_reported'] += 1
        for k in h.taken:
            if k not in self.g_pending_report:
                self.fail('I11', 'abandoned report for %s seen twice' % kstr(k))
            self.g_pending_report.discard(k)
        if h.mode == X:
            for k in h.keys:
                if k in self.abandoned:
                    self.fail('I11', 'X body on %s started while its abandoned flag is still set' % kstr(k))

    def drop_hold(self, hid, abandon):
        h = self.holds[hid]
        if not h.body_started:
            self._abandon_add(h.taken)
        elif abandon and h.mode == X:
            self._abandon_add(h.keys)
            self.g_pending_report |= set(h.keys)
        self.recs[h.pid].holds.discard(hid)
        self._trie_remove_hold(hid, h)
        del self.holds[hid]
        return set(h.keys)

    def enqueue(self, T):
        self.queue.add(T.seq)
        self._trie_enqueue(T)

    def dequeue(self, T):
        self.queue.discard(T.seq)
        self._trie_dequeue(T)

    def maybe_drop_rec(self, P):
        r = self.recs.get(P)
        if r is not None and not r.holds and r.ticket is None:
            del self.recs[P]

    # -- operation framing ------------------------------------------------------------------------
    def _begin(self):
        self.N = []
        self.soft = []
        self._start_inh = dict((P, r.inh) for P, r in self.recs.items())
        self._start_graph = self.graph()
        self._seeds = set()
        self._lending_before = self.lending()

    # -- 4.2 / 4.3 ----------------------------------------------------------------------------------
    def re_examine(self, region):
        G = set()
        snapshot = sorted(self.queue)
        for t in snapshot:
            T = self.tickets[t]
            if T.state != 'queued':
                continue
            if 'region-holds-only' in self.mutation and getattr(self, '_region_is_ticket', False):
                continue
            if not any(overlap(k, r) for k in T.keys for r in region):
                continue
            if self.grantable_t(T):
                self.dequeue(T)
                T.hold = self.new_hold(T.pid, T.keys, T.mode, False, T.seq)
                T.state = 'granted'
                self.N.append((T.pid[0], ('wake', T.pid[1], T.seq)))
                G.add(T.pid)
        return G

    def finish(self, region, seeds, skip):
        G = self.re_examine(region)
        for P in sorted(seeds | G):
            self.maybe_drop_rec(P)
        if 'no-granted-seeds' not in self.mutation:
            self._seeds |= G
        self.relevel_all(skip)

    def relevel_all(self, skip):
        if 'no-relevel-cancel' in self.mutation and getattr(self, '_in_cancel', False):
            return
        start = self._start_inh
        g = self.graph()
        pred = self.pred_of(g)
        lending = self.lending()
        changed = []
        posted = []
        for Xp in sorted(set(start) | set(self.recs)):
            old = start.get(Xp, INF)
            new = self.inh_formula(Xp, pred, lending) if Xp in self.recs else INF
            if Xp in self.recs:
                self.recs[Xp].inh = new
            if new != old:
                changed.append(Xp)
                # literal, left to right: `new != old and X not in skip and applies(X)`
                if Xp not in skip and self.applies(Xp):
                    r = self.recs.get(Xp)
                    if self.coalesce and r is not None and r.rl:
                        STATS['relevel_coalesced'] += 1
                    else:
                        if self.coalesce and r is not None:
                            r.rl = True
                        self.N.append((Xp[0], ('relevel', Xp[1])))
                        posted.append(Xp)
        self._impl_note_check(changed, posted, g, lending)

    def _impl_note_check(self, changed, posted, g_after, lending):
        """4.3's implementation note, its four lending cases, against the reference loop."""
        before = self._lending_before
        if not before and not lending:
            # it does nothing: every inh must already be INF and stay so
            bad = list(changed) + [Xp for Xp, r in self.recs.items() if r.inh != INF]
            if bad:
                self.fail('impl-note', 'lending false before and after, but the reference changed '
                          'or holds a non-INF inh for %r' % (sorted(set(bad)),))
            return
        if before and not lending:
            # it sets every rec's inh to INF and posts nothing
            bad = [Xp for Xp, r in self.recs.items() if r.inh != INF]
            if bad or posted:
                self.fail('impl-note', 'lending true before, false after: reference inh %r, posts %r; '
                          'the note sets INF and posts nothing'
                          % ([(Xp, self.recs[Xp].inh) for Xp in bad], posted))
            return
        if not before and lending:
            return          # it runs the reference loop over every rec
        seeds = self._seeds
        A = set(seeds) | self.reach_depth(g_after, seeds, 8)
        if 'impl-note-after-only' not in self.mutation:
            A |= self.reach_depth(self._start_graph, seeds, 8)
        missing = [Xp for Xp in changed if Xp not in A]
        if missing:
            self.fail('impl-note', 'inh of %r changed but they are not in A (seeds %r)'
                      % (missing, sorted(seeds)))

    # -- 4.4 ----------------------------------------------------------------------------------------
    def attach_home(self, H, prio, port, progress):
        self._begin()
        self.require(H not in self.homes, 'attach twice')
        self.require(not (self.one_home and self.homes), 'attach_home: before L3 a manager accepts one home')
        self.homes[H] = [prio, port, progress]
        self.cache_refs[H] = {}     # rev 5 (4.4): empty; detach_home deletes it (4.11)
        self._seeds = set()     # no request, hold or base changed (false->true runs the full loop)
        self.relevel_all(set())
        return 'ok', self.N

    # -- 4.5 ----------------------------------------------------------------------------------------
    def try_acquire(self, P, info, keys, mode, refs=((), ())):
        self._begin()
        r = self.recs.get(P)
        self.require(r is None or r.ticket is None, 'try_acquire: P has a ticket')
        self._apply_refs(P[0], refs)        # 4.15: a miss or an eviction, same critical section
        if not self.grantable(P, keys, mode, INF):
            return ('busy',), []
        self.ensure_rec(P, info)
        hid = self.new_hold(P, keys, mode, True, INF)
        h = self.holds[hid]
        return ('granted', hid, h.gen, bool(h.taken)), []

    # -- 4.6 ----------------------------------------------------------------------------------------
    def block(self, P, info, keys, mode, refs=((), ())):
        self._begin()
        r = self.recs.get(P)
        self.require(r is None or r.ticket is None, 'block: P has a ticket')
        self._apply_refs(P[0], refs)
        self.ensure_rec(P, info)
        self.next_seq += 1
        s = self.next_seq
        if self.grantable(P, keys, mode, INF):
            hid = self.new_hold(P, keys, mode, False, s)
            self.tickets[s] = Ticket(P, keys, mode, s, 'granted', hold=hid)
            self.recs[P].ticket = s
            return (s, 'granted'), []
        targets = (set(h.pid for h in self.blk_hold(P, keys, mode))
                   | set(U.pid for U in self.blk_wait(P, keys, mode, INF)))
        if P in self.reach_in(self.graph(), targets):
            cycle = self.deadlock_cycle(P, keys, mode, targets)
            STATS['deadlock_len_%d' % len(cycle)] += 1
            self._g_check_deadlock(P, keys, mode, s, cycle)
            self.tickets[s] = Ticket(P, keys, mode, s, 'failed', error=('DeadlockError', tuple(cycle)))
            self.recs[P].ticket = s
            return (s, 'failed'), []
        T = Ticket(P, keys, mode, s, 'queued')
        self.tickets[s] = T
        self.recs[P].ticket = s
        self.enqueue(T)
        self._seeds = {P}
        self.relevel_all(set())
        return (s, 'queued'), self.N

    def _edge_blockers(self, A, keys, mode, s, B):
        out = [h.keys for h in self.blk_hold(A, keys, mode) if h.pid == B]
        out += [U.keys for U in self.blk_wait(A, keys, mode, s) if U.pid == B]
        return out

    def deadlock_cycle(self, P, keys, mode, targets):
        visited = set()
        found = [None]

        def dfs(A, path):
            if A in visited:
                return False
            visited.add(A)
            path.append(A)
            for B in sorted(self.succ_of(A)):
                if B == P:
                    found[0] = list(path)
                    return True
                if dfs(B, path):
                    return True
            path.pop()
            return False

        for t in sorted(targets):
            if dfs(t, []):
                break
        path = found[0]
        if path is None:
            self.fail('I7', 'deadlock_cycle found no path back to %r' % (P,))
        nodes = [P] + path
        owner = self.recs[P].owner
        cyc = []
        for i, A in enumerate(nodes):
            B = nodes[(i + 1) % len(nodes)]
            if A == P:
                akeys, amode, aseq = keys, mode, INF
            else:
                T = self.tickets[self.recs[A].ticket]
                akeys, amode, aseq = T.keys, T.mode, T.seq
            bl = self._edge_blockers(A, akeys, amode, aseq, B)
            key = None
            for k in akeys:
                if any(overlap(k, b) for bk in bl for b in bk):
                    key = k
                    break
            same = A[0] == P[0] and self.recs[A].owner == owner     # rev 4: home and owner
            cyc.append((A[1] if same else None, key, A))
        return cyc

    def _g_check_deadlock(self, P, keys, mode, s, cycle):
        """Every DeadlockError corresponds to a real cycle at that moment."""
        T = Ticket(P, keys, mode, s, 'queued')
        self.tickets[s] = T
        old = self.recs[P].ticket
        self.recs[P].ticket = s
        self.queue.add(s)
        try:
            g = self.graph()
            if P not in self.reach_in(g, g.get(P, set())):
                self.fail('I7', 'DeadlockError for %r but no cycle through it' % (P,))
            nodes = [c[2] for c in cycle]
            if nodes[0] != P:
                self.fail('I7', 'cycle does not start with the requester')
            for i, A in enumerate(nodes):
                B = nodes[(i + 1) % len(nodes)]
                if B not in g.get(A, set()):
                    self.fail('I7', 'cycle edge %r -> %r does not exist' % (A, B))
                if cycle[i][1] is None:
                    self.fail('I7', 'cycle edge %r -> %r has no key' % (A, B))
        finally:
            self.queue.discard(s)
            del self.tickets[s]
            self.recs[P].ticket = old

    # -- 4.7 ----------------------------------------------------------------------------------------
    def resume(self, P, tid):
        self._begin()
        T = self.tickets.get(tid)
        self.require(T is not None and T.pid == P and self.recs[P].ticket == tid, 'resume: not P\'s ticket')
        if T.state == 'queued':
            return ('queued',), []
        if T.state == 'granted':
            h = self.holds[T.hold]
            out = ('granted', T.hold, h.gen, bool(h.taken))
        elif T.state == 'cancelled':
            out = ('TimeoutError',)
        else:
            out = T.error
        del self.tickets[tid]
        self.recs[P].ticket = None
        self.maybe_drop_rec(P)
        return out, []

    # -- 4.8 ----------------------------------------------------------------------------------------
    def start_body(self, P, hid):
        self._begin()
        h = self.holds.get(hid)
        self.require(h is not None and h.pid == P, 'start_body: not P\'s hold')
        h.body_started = True
        self._g_body_start(h)
        return []

    # -- 4.9 ----------------------------------------------------------------------------------------
    def cancel(self, P, tid):
        self._begin()
        T = self.tickets.get(tid)
        if T is None or T.pid != P:
            return None, []
        if T.state != 'queued':
            return T.state, []
        self.dequeue(T)
        T.state = 'cancelled'
        if 'cancel-marks-failed' in self.mutation:
            T.state, T.error = 'failed', ('TimeoutError',)
        self._seeds = {P}
        self._region_is_ticket = True
        self._in_cancel = True
        try:
            self.finish(set(T.keys), {P}, set())
        finally:
            self._region_is_ticket = False
            self._in_cancel = False
        return 'cancelled', self.N

    # -- 4.10 ---------------------------------------------------------------------------------------
    def release(self, P, hid):
        self._begin()
        h = self.holds.get(hid)
        if h is None or h.pid != P:
            return []
        region = self.drop_hold(hid, False)
        self._seeds = {P}
        self.finish(region, {P}, set())
        return self.N

    # -- 4.11 ---------------------------------------------------------------------------------------
    def end_locked(self, P, abandon):
        rec = self.recs.get(P)
        if rec is None:
            return set()
        region = set()
        if rec.ticket is not None:
            T = self.tickets[rec.ticket]
            if T.state == 'queued':
                self.dequeue(T)
                region |= set(T.keys)
            del self.tickets[rec.ticket]
            rec.ticket = None
        for hid in sorted(rec.holds):
            region |= self.drop_hold(hid, abandon)
        del self.recs[P]
        return region

    def end(self, P, abandon):
        self._begin()
        self._seeds = {P}
        region = self.end_locked(P, abandon)
        self.finish(region, set(), {P})
        return self.N

    def detach_home(self, H):
        self._begin()
        # rev 5 (4.11): the caller, ~lock_home, closed H's port first (5.7); nothing here reads
        # it: the closed assertion is the world's destroy action's, outside the operation
        self.require(H in self.homes, 'detach_home: H%s is not attached' % H)
        if 'no-unref-on-detach' not in self.mutation:
            for k, c in sorted(self.cache_refs[H].items()):    # a copy of the multiset
                for _ in range(c):
                    self._unref(H, k)
            self.cache_refs.pop(H, None)    # delete M.cache_refs[H] (gone already under 'unref-pops-entry')
        region = set()
        ended = set()
        for P in sorted(list(self.recs)):                    # a copy: end_locked deletes recs
            if P[0] == H:
                region |= self.end_locked(P, True)
                ended.add(P)
        del self.homes[H]
        self._seeds = set(ended)
        self.finish(region, set(), ended)
        for (h, post) in self.N:
            if h == H:
                self.fail('detach', 'a notification of detach_home targets the detached home')
        return self.N

    # -- 4.12 ---------------------------------------------------------------------------------------
    def set_base(self, P, base):
        self._begin()
        if P not in self.recs:
            return []
        self.recs[P].base = base
        self._seeds = {P}
        self.relevel_all(set())
        return self.N

    # -- 4.13 ---------------------------------------------------------------------------------------
    def blocking_sinks(self, P):
        # 4.13 (rev 5): { (Q, M.homes[Q.home].progress.load()) : Q in reach({P}), Q != P,
        #                 succ(Q) == {} }
        g = self.graph()
        return set((Q, self.homes[Q[0]][2].load()) for Q in self.reach_in(g, {P})
                   if Q != P and not self.succ_of(Q))

    def query(self, pre, viewer):
        """4.13 lock-query (rev 5): { held, waiting } for the keys under `pre` (() is the root);
        viewer = (home, owner). Only pids are filtered by visibility; counts include every hold."""
        def visible(Q):
            return Q[0] == viewer[0] and Q in self.recs and self.recs[Q].owner == viewer[1]
        keys = set()
        for h in self.holds.values():
            keys |= set(k for k in h.keys if prefix(pre, k))
        held = []
        for k in sorted(keys):
            hs = [h for i, h in sorted(self.holds.items()) if k in h.keys]
            xs = [h for h in hs if h.mode == X]
            # 4.13 (rev 5b): the largest gen among the X holds with k, the latest X grant on k (one
            # process may hold two: X a.b, then a nested X {a.b, b}, since its own holds never block
            # it); mutation 'query-first-gen' shows the first
            pick = min if 'query-first-gen' in self.mutation else max
            held.append({'key': k, 'mode': X if xs else S,
                         'pids': [h.pid[1] for h in hs if visible(h.pid)],
                         'count': len(hs), 'generation': pick(h.gen for h in xs) if xs else None,
                         'abandoned': k in self.abandoned})
        waiting = []
        for t in sorted(self.queue):
            T = self.tickets[t]
            if any(prefix(pre, k) for k in T.keys):
                waiting.append({'key': T.keys, 'mode': T.mode,
                                'pid': T.pid[1] if visible(T.pid) else None})
        return {'held': held, 'waiting': waiting}

    def read_inh(self, P):
        r = self.recs.get(P)
        if r is None:
            return INF
        if 'coalesce-no-clear' not in self.mutation:
            r.rl = False        # 4.3 coalescing: the apply clears the flag under M.mu
        return r.inh

    def peek_inh(self, P):
        # the checker's read of read_inh's value, without the apply's side effect (rev 5)
        r = self.recs.get(P)
        return INF if r is None else r.inh

    # -- invariants over the manager alone ------------------------------------------------------------
    def check(self):
        # I1
        hl = list(self.holds.items())
        for i in range(len(hl)):
            for j in range(i + 1, len(hl)):
                a, b = hl[i][1], hl[j][1]
                if a.pid != b.pid and conflicts(a.keys, a.mode, b.keys, b.mode):
                    self.fail('I1', 'holds %d and %d of %r and %r conflict' % (hl[i][0], hl[j][0], a.pid, b.pid))
        # I2
        q = set(t for t, T in self.tickets.items() if T.state == 'queued')
        if q != self.queue:
            self.fail('I2', 'queue %r != queued tickets %r' % (sorted(self.queue), sorted(q)))
        owned = {}
        for P, r in self.recs.items():
            if r.ticket is not None:
                if r.ticket in owned:
                    self.fail('I2', 'ticket owned twice')
                owned[r.ticket] = P
        if set(owned) != set(self.tickets):
            self.fail('I2', 'tickets %r are not exactly the recs\' tickets %r' % (sorted(self.tickets), sorted(owned)))
        for t, T in self.tickets.items():
            if owned[t] != T.pid:
                self.fail('I2', 'ticket %d pid mismatch' % t)
            if T.state == 'granted':
                h = self.holds.get(T.hold)
                if h is None or h.pid != T.pid or h.body_started:
                    self.fail('I2', 'granted ticket %d has no unstarted hold of its process' % t)
        # I3
        for P, r in self.recs.items():
            if not r.holds and r.ticket is None:
                self.fail('I3', 'rec %r with nothing' % (P,))
            if r.holds != set(i for i, h in self.holds.items() if h.pid == P):
                self.fail('I3', 'rec %r holds mismatch' % (P,))
        for i, h in self.holds.items():
            if h.pid not in self.recs:
                self.fail('I3', 'hold %d without rec' % i)
        # I9 (rev 5): M.cache_refs has an entry exactly for each attached home; I4: counts positive
        for H, d in self.cache_refs.items():
            if H not in self.homes:
                self.fail('I9', 'key-cache references of detached home H%s kept: %s'
                          % (H, sorted(kstr(k) for k in d)))
            if any(c <= 0 for c in d.values()):
                self.fail('I4', 'non-positive cache reference count for H%s' % H)
        for H in self.homes:
            if H not in self.cache_refs:
                self.fail('I9', 'attached home H%s has no key-cache entry' % H)
        # 4.13: every process with a rec is on an attached home (detach_home ends a home's recs
        # before it deletes the home), so blocking_sinks' load reaches only attached homes
        for P in self.recs:
            if P[0] not in self.homes:
                self.fail('4.13', 'rec %r of a home that is not attached' % (P,))
        self.check_trie()
        # I6 first half
        for t in sorted(self.queue):
            if self.grantable_t(self.tickets[t]):
                self.fail('I6', 'queued ticket %d of %r is grantable after the operation' % (t, self.tickets[t].pid))
        # I7
        g = self.graph()
        cyc = self.find_cycle(g)
        if cyc:
            self.fail('I7', 'wait-for cycle %r' % (cyc,))
        # I8 first half, against an independent enumeration of paths of 1..8 edges
        lending = self.lending()
        ref = self.inh_reference(g, lending)
        for P, r in self.recs.items():
            if r.inh != ref[P]:
                self.fail('I8', 'rec %r inh %s != transitive definition %s' % (P, r.inh, ref[P]))
        # I11
        seen = set(self.abandoned)
        for h in self.holds.values():
            if seen & h.taken:
                self.fail('I11', 'abandoned flags not disjoint')
            seen |= h.taken
        expect = set(self.abandoned)
        for h in self.holds.values():
            if not h.body_started:
                expect |= h.taken
        if expect != self.g_pending_report:
            self.fail('I11', 'reports not conserved: pending %r, flags+unstarted taken %r'
                      % (sorted(self.g_pending_report), sorted(expect)))
        for h in self.holds.values():
            if h.mode == S and h.taken:
                self.fail('I11', 'S hold took a flag')

    def inh_reference(self, g, lending):
        """3.3 by brute force: walk every path backwards from each X, 1 to 8 edges."""
        out = {}
        pred = {}
        for a, bs in g.items():
            for b in bs:
                pred.setdefault(b, []).append(a)
        for Xp in self.recs:
            best = INF
            if lending:
                anc = set()
                stack = [(Xp, 0)]
                while stack:
                    n, d = stack.pop()
                    if d == 8:
                        continue
                    for y in pred.get(n, ()):
                        if y != Xp:
                            anc.add(y)
                        stack.append((y, d + 1))
                for Y in anc:
                    if self.inheritance and Y[0] in self.homes and self.homes[Y[0]][0]:
                        best = min(best, self.recs[Y].base)
            out[Xp] = best
        return out

    def check_trie(self):
        exp = {}

        def node(k):
            if k not in exp:
                exp[k] = Node()
            return exp[k]
        node(())
        for i, h in self.holds.items():
            for k in h.keys:
                node(k).here[h.mode].add(i)
                for j in range(len(k) + 1):
                    node(k[:j]).sub[h.mode] += 1
        for t in self.queue:
            T = self.tickets[t]
            for k in T.keys:
                node(k).wq.add(t)
                for j in range(len(k) + 1):
                    node(k[:j]).subw += 1
        for k in self.abandoned:
            node(k).ab = True
        for H, d in self.cache_refs.items():
            for k, c in d.items():
                node(k).refs += c
                for j in range(len(k)):
                    node(k[:j])
        for k, e in exp.items():
            n = self.trie.get(k)
            if n is None:
                if not e.empty():
                    self.fail('I4', 'trie node %s missing' % kstr(k))
                continue
            if (n.here != e.here or n.sub != e.sub or n.wq != e.wq or n.subw != e.subw or n.ab != e.ab
                    or n.refs != e.refs):
                self.fail('I4', 'trie node %s counts differ' % kstr(k))
        for k, n in self.trie.items():
            if k == ():
                continue
            e = exp.get(k)
            if e is None or e.empty():
                if not n.empty():
                    self.fail('I4', 'trie node %s has stale counts' % kstr(k))
                if not n.children:
                    self.fail('I4', 'empty trie node %s kept' % kstr(k))
                # 2.4: an all-zero node is kept while it has a child (the abandoned flag is
                # counted in no subtree count); I4 allows it
        for k, n in self.trie.items():
            for c in n.children:
                if c not in self.trie or c[:-1] != k:
                    self.fail('I4', 'trie child link %s -> %s broken' % (kstr(k), kstr(c)))
            if k != () and k not in self.trie[k[:-1]].children:
                self.fail('I4', 'trie node %s not linked from its parent' % kstr(k))
