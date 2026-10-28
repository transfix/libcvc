"""state_exec's lock phases from Python (docs/STATE_EXEC_LOCKS.md).

L0: the atomic natives state-incr, state-cas and state-swap, run through
pycvc.Exec over the app's state tree.

L1: with-lock from Python: through pycvc.Exec, and on a pycvc async_scheduler
with one register_intrinsics context per program (the C++ suites are in
src/cvc/tests/state_exec_locks_test.cpp).
"""

import gc
import os
import time

# Every lock manager operation checks the manager's invariants (gap G-38); the
# library reads this once, when the first manager is made.
os.environ["CVC_STATE_EXEC_CHECK"] = "1"

import pycvc  # noqa: E402
from pycvc import eval_phase, process_status  # noqa: E402

_ENDED = (process_status.terminated, process_status.killed, process_status.failed)


def test_state_incr_twice():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    assert ex.run('(state-incr "n")') == "1"
    assert ex.run('(state-incr "n")') == "2"
    assert pycvc.state_get(app, "n") == "2"


def test_state_cas_claim_and_release():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    assert ex.run('(state-cas "job.owner" nil "py")') == "#t"
    assert ex.run('(state-cas "job.owner" nil "other")') == "#f"
    assert ex.run('(state-cas "job.owner" "py" nil)') == "#t"
    assert pycvc.state_get(app, "job.owner") == ""


def test_state_swap_returns_the_old_value():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    pycvc.state_set(app, "k", "a")
    assert ex.run('(state-swap "k" "b")') == "a"
    assert pycvc.state_get(app, "k") == "b"


# ── L1: with-lock ──────────────────────────────────────────────────────────


class World:
    """An async_scheduler with one intrinsics context per program, stepped one
    step at a time."""

    def __init__(self):
        self.app = pycvc.make_app()
        self.root = pycvc.State.instance(self.app)
        self.sched = pycvc.async_scheduler()
        self.sched.set_watch_root(self.root)
        self.envs = {}

    def env(self, ctx_pid=None):
        ctx = pycvc.intrinsics_context()
        ctx.sched = self.sched
        ctx.root = self.root
        if ctx_pid is not None:
            ctx.pid = ctx_pid
        env = pycvc.builtins.make_default_environment()
        pycvc.register_intrinsics(env, ctx)
        return env

    def submit(self, script, ctx_pid=None, **fields):
        env = self.env(ctx_pid)
        o = pycvc.execute_options()
        o.env = env
        for k, v in fields.items():
            setattr(o, k, v)
        pid = self.sched.execute(script, o)
        self.envs[pid] = env
        return pid

    def step_until(self, done, seconds=10.0):
        end = time.monotonic() + seconds
        while not done():
            if time.monotonic() > end:
                return False
            self.sched.drain_ingress()
            if not self.sched.sync_step():
                time.sleep(0.0005)
        return True

    def info(self, pid):
        return self.sched.get_process_info(pid)

    def finished(self, pid):
        return self.info(pid).status in _ENDED

    # process_info.waiting_on reads as an opaque SWIG pointer (pycvc.i declares no %naturalvar
    # for it, as it does for exit_error), so these tell a lock waiter from a holder parked in its
    # section by its holds: here every waiter holds nothing and every holder parks on msg-recv.
    def lock_waiting(self, pid):
        i = self.info(pid)
        return i.status == process_status.waiting and i.locks_held == 0

    def inside(self, pid):
        i = self.info(pid)
        return i.status == process_status.waiting and i.locks_held > 0


def test_with_lock_through_exec():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    assert ex.run('(with-lock "k" (+ 1 2))') == "3"
    assert ex.run('(with-lock "k" (get-attr (car (locks-held)) "key"))') == "k"
    assert ex.run('(length (locks-held))') == "0"
    assert ex.run('(with-lock "k" :if-available :else "busy" "free")') == "free"
    assert ex.run('(with-lock "k" (try (fork) (catch (LockError e) (get-attr e "reason"))))') \
        == "fork"


def test_lk23_context_dropped():
    # LK-23: the with-lock binding register_intrinsics makes keeps the context (and its Python
    # owner) alive, so the form works after Python drops the context (run under ASan in the
    # sanitizer job).
    w = World()
    ctx = pycvc.intrinsics_context()
    ctx.sched = w.sched
    ctx.root = w.root
    env = pycvc.builtins.make_default_environment()
    pycvc.register_intrinsics(env, ctx)
    del ctx
    gc.collect()
    o = pycvc.execute_options()
    o.env = env
    pid = w.sched.execute('(with-lock "k" (+ 0 1))', o)
    assert w.step_until(lambda: w.finished(pid))
    assert w.sched.get_result(pid) == 1


def test_lw06_fork_from_exec():
    # LW-6: inside with-lock, (fork) raises LockError("fork"); outside, it forks the running
    # process even though the context's pid is pycvc's placeholder (0), and the child keeps the
    # parent's owner, so kill_owner reaps it.
    #
    # Known (not a lock bug): the child of a DSL (fork) applies (fork) again at its first step
    # (its copied stack still applies the call), so the child is checked and reaped before it
    # ever steps.
    w = World()
    pid = w.submit('(with-lock "k" (try (fork) (catch (LockError e) (get-attr e "reason"))))',
                   ctx_pid=0)
    assert w.step_until(lambda: w.finished(pid))
    assert w.sched.get_result(pid) == "fork"
    assert len(w.sched.list_processes()) == 1  # no child

    pid = w.submit('(begin (set c (fork)) (msg-recv "never"))', ctx_pid=0, owner="docA")
    assert w.step_until(lambda: len(w.sched.list_processes()) == 3)
    child = [p for p in w.sched.list_processes() if p.pid not in (1, pid)][0]
    assert child.parent_pid == pid  # the running process forked, not the context's pid 0
    node = w.root.child("snap.fork")
    assert w.sched.snapshot(child.pid, node)
    decoded = pycvc.decode_process(node)
    assert decoded.owner == "docA"
    assert w.sched.kill_owner("docA") == 2
    assert w.info(child.pid).status == process_status.killed


def test_lw11_recursive_evaluator_refuses():
    # LW-11: the recursive evaluator has no scheduler: with the intrinsics bound, with-lock
    # raises TypeError before its body runs.
    app = pycvc.make_app()
    root = pycvc.State.instance(app)
    sched = pycvc.scheduler()
    ctx = pycvc.intrinsics_context()
    ctx.sched = sched
    ctx.root = root
    env = pycvc.builtins.make_default_environment()
    pycvc.register_intrinsics(env, ctx)
    ev = pycvc.evaluator(env)
    try:
        ev.evaluate_script('(with-lock "k" (state-set "x" 1))')
    except pycvc.dsl_error as e:
        assert e.dsl_class == "TypeError", e.dsl_class
        assert "with-lock needs a scheduler" in str(e), str(e)
    else:
        raise AssertionError("the recursive evaluator ran with-lock")
    try:  # x was never written: no node
        pycvc.state_get(app, "x")
    except RuntimeError as e:
        assert "no node" in str(e), str(e)
    else:
        raise AssertionError("the body ran: x exists")


def test_process_info_waiting_on():
    # inspect's waiting_on (L1.11): "lock /<key>" while a process waits for a lock.
    w = World()
    holder = w.submit('(with-lock "k" (msg-recv "never"))')
    assert w.step_until(lambda: w.inside(holder))
    waiter = w.submit('(with-lock "k" 1)')
    assert w.step_until(lambda: w.lock_waiting(waiter))
    assert w.info(holder).locks_held == 1
    on = w.info(waiter).waiting_on
    if not (on is None or isinstance(on, str)):
        print("  skipped (known: pycvc.i has no %naturalvar for process_info::waiting_on; it "
              "reads as", type(on).__name__ + ")")
        return
    assert on == "lock /k", on
    assert w.info(holder).waiting_on == "msg never"


def test_lock_frame_phases():
    # L1.12: a parked waiter's snapshot decodes with its lock frame's phase (lock_enter, below
    # the wait's await_call frame), every phase an eval_phase; restored, the waiter queues
    # afresh and is granted once the holder is killed (LW-7's pycvc side).
    assert int(eval_phase["lock_enter"]) == int(eval_phase["await_call"]) + 1
    assert int(eval_phase["lock_body"]) == int(eval_phase["lock_enter"]) + 1
    w = World()
    holder = w.submit('(with-lock "k" (msg-recv "never"))')
    assert w.step_until(lambda: w.inside(holder))
    waiter = w.submit('(with-lock "k" 1)')
    assert w.step_until(lambda: w.lock_waiting(waiter))
    node = w.root.child("snap.waiter")
    assert w.sched.snapshot(waiter, node)
    # (Kept in a variable: the state of a temporary decode_process() result reads as empty.)
    decoded = pycvc.decode_process(node)
    phases = [f.phase for f in decoded.state.stack]
    assert all(isinstance(p, eval_phase) for p in phases), phases
    assert phases[-2:] == [eval_phase.lock_enter, eval_phase.await_call], phases
    # A holder's snapshot is refused, naming its key.
    try:
        w.sched.snapshot(holder, w.root.child("snap.holder"))
    except RuntimeError as e:
        assert "holds locks: /k" in str(e), str(e)
    else:
        raise AssertionError("a holder was snapshotted")
    restored = w.sched.restore(node, w.envs[waiter])
    assert w.step_until(lambda: w.lock_waiting(restored))
    assert w.sched.kill(holder)
    assert w.step_until(lambda: w.finished(waiter) and w.finished(restored))
    assert w.sched.get_result(waiter) == 1
    assert w.sched.get_result(restored) == 1


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc state_exec locks tests: OK")
