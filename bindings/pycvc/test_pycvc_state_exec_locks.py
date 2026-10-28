"""state_exec's lock phases from Python (docs/STATE_EXEC_LOCKS.md).

L0: the atomic natives state-incr, state-cas and state-swap, run through
pycvc.Exec over the app's state tree.

L1: with-lock from Python: through pycvc.Exec, and on a pycvc async_scheduler
with one register_intrinsics context per program (the C++ suites are in
src/cvc/tests/state_exec_locks_test.cpp).

L2: Exec's policy (Exec(app, policy=None, idle_timeout=60.0), PR-5), its
set_priority and get_priority from any thread (PR-8, design 7.4), and
pycvc.set_thread_class (PR-9, design 7.6). PR-8's stress cases run
CVC_PYCVC_LOCKS_RUNS times each (500 by default).
"""

import gc
import math
import os
import sys
import threading
import time

# Every lock manager operation checks the manager's invariants (gap G-38); the
# library reads this once, when the first manager is made.
os.environ["CVC_STATE_EXEC_CHECK"] = "1"

import pycvc  # noqa: E402
from pycvc import eval_phase, process_status  # noqa: E402

_ENDED = (process_status.terminated, process_status.killed, process_status.failed)

RUNS = int(os.environ.get("CVC_PYCVC_LOCKS_RUNS", "500"))
# A case's outcome must come within this many seconds (a hang fails it); the times are printed.
WAIT = 10.0


def _raises(exc, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc as e:
        return e
    raise AssertionError(f"{fn!r} must raise {exc.__name__}")


def _join(thread):
    thread.join(WAIT)
    assert not thread.is_alive(), f"{thread.name} did not finish within {WAIT} s"


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

    # These tell a lock waiter from a holder parked in its section by its holds: here every waiter
    # holds nothing and every holder parks on msg-recv (test_process_info_waiting_on reads
    # waiting_on itself).
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


# ── L2: the policy (PR-5) ───────────────────────────────────────────────


def test_pr05_exec_policy():
    sp = pycvc.scheduling_policy
    # policy=None (or none given) reads state_exec.policy when the scheduler is built; unset, it
    # is round_robin, today's.
    app = pycvc.make_app()
    assert pycvc.Exec(app).scheduler().policy() == sp.round_robin
    pycvc.state_set(app, "state_exec.policy", "priority_rr")
    assert pycvc.Exec(app).scheduler().policy() == sp.priority_rr
    assert pycvc.Exec(app, policy=None).scheduler().policy() == sp.priority_rr
    # An explicit policy is the Exec's whatever the setting says.
    assert pycvc.Exec(app, policy="round_robin").scheduler().policy() == sp.round_robin
    # An unknown setting is logged, and round_robin used.
    pycvc.state_set(app, "state_exec.policy", "fifo")
    assert pycvc.Exec(app).scheduler().policy() == sp.round_robin

    # With the setting unset: by keyword, by name and by member. SWIG ignores a kwargs feature
    # that names nothing, so this is what checks it took.
    app = pycvc.make_app()
    assert pycvc.Exec(app, policy="priority_rr").scheduler().policy() == sp.priority_rr
    assert pycvc.Exec(app, policy=sp.priority_rr).scheduler().policy() == sp.priority_rr
    assert pycvc.Exec(app, policy="priority", idle_timeout=30.0).scheduler().policy() == sp.priority
    assert pycvc.Exec(app, idle_timeout=30.0, policy=sp.priority).scheduler().policy() == sp.priority
    assert pycvc.Exec(app, sp.priority_rr, 30.0).scheduler().policy() == sp.priority_rr
    assert pycvc.Exec(app=app).scheduler().policy() == sp.round_robin

    # Anything else is a ValueError, and so is a negative or NaN idle_timeout.
    for bad in ("fifo", "", "PRIORITY", 2, 2.0, True, [], pycvc.process_status.ready):
        e = _raises(ValueError, pycvc.Exec, app, policy=bad)
        assert "policy" in str(e)
    for bad in (-1.0, math.nan):
        _raises(ValueError, pycvc.Exec, app, idle_timeout=bad)
    _raises(TypeError, pycvc.Exec, app, polcy="priority")  # a misspelt keyword

    # idle_timeout replaces run()'s 60 s: a program parked with nothing pending is killed after it.
    ex = pycvc.Exec(app, idle_timeout=0.2)
    t0 = time.monotonic()
    e = _raises(RuntimeError, ex.run, '(begin (msg-recv "pr05.nobody") 1)')
    elapsed = time.monotonic() - t0
    print(f"  idle_timeout=0.2: killed after {elapsed:.2f} s")
    assert "parked with no pending work" in str(e)
    assert elapsed < WAIT


# ── L2: set_priority and get_priority (PR-8) ────────────────────────────


def _info(base, floor=0, policy=pycvc.scheduling_policy.round_robin):
    return {"base": base, "effective": base, "floor": floor, "policy": policy}


def test_pr08_priority_calls_on_an_idle_exec():
    # No run() is active: a call acts at once, from this thread or another.
    app = pycvc.make_app()
    ex = pycvc.Exec(app, policy="priority_rr")
    pid = ex.scheduler().execute("(sleep 100)")
    assert ex.get_priority(pid) == _info(0, policy=pycvc.scheduling_policy.priority_rr)
    assert ex.set_priority(pid, 7) is True
    assert ex.get_priority(pid)["base"] == 7
    assert ex.scheduler().get_process_info(pid).priority == 7
    assert ex.get_priority(99999) is None
    assert ex.set_priority(99999, 1) is False
    out = {}

    def other():
        out["applied"] = ex.set_priority(pid, 3)
        out["info"] = ex.get_priority(pid)

    t = threading.Thread(target=other, name="pr08-idle")
    t.start()
    _join(t)
    assert out["applied"] is True
    assert out["info"]["base"] == 3


def test_pr08_priority_calls_from_another_thread_while_run_pumps():
    # The monitor calls while run() pumps on this thread: run() answers each call between its
    # steps, so the program sees the change at its next step.
    app = pycvc.make_app()
    ex = pycvc.Exec(app, policy="priority_rr")
    started = threading.Event()
    done = threading.Event()
    seen = {}

    def on_start():
        seen["pid"] = ex.scheduler().current_pid()  # the run() thread, inside the step
        started.set()
        return True

    def monitor():
        if not started.wait(WAIT):
            return
        pid = seen["pid"]
        seen["before"] = ex.get_priority(pid)
        seen["applied"] = ex.set_priority(pid, 9)
        seen["after"] = ex.get_priority(pid)
        done.set()

    deadline = time.monotonic() + WAIT
    ex.register_fn("started", on_start)
    ex.register_fn("monitor-done", lambda: done.is_set() or time.monotonic() > deadline)
    ex.register_fn("my-base", lambda: ex.get_priority(seen["pid"])["base"])
    t = threading.Thread(target=monitor, name="pr08-monitor")
    t.start()
    out = ex.run("(begin (started) (while (not (monitor-done)) (sleep 0.005)) (my-base))")
    _join(t)
    assert done.is_set()
    assert seen["before"]["base"] == 0
    assert seen["applied"] is True
    assert seen["after"]["base"] == 9
    assert out == "9"


def test_pr08_registered_function_calls_get_priority():
    # From inside the program (a registered function or coroutine, on the run() thread): answered
    # at once, no wait for run() itself.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)

    def mine():
        pid = ex.scheduler().current_pid()
        assert ex.set_priority(pid, 4) is True
        return ex.get_priority(pid)["base"]

    async def mine_later(pid):
        assert ex.set_priority(pid, 6) is True
        return ex.get_priority(pid)["base"]

    ex.register_fn("mine", mine)
    ex.register_async_coro("mine-later", mine_later)
    t0 = time.monotonic()
    assert ex.run("(mine)") == "4"
    assert ex.run("(mine-later (self-pid))") == "6"
    assert time.monotonic() - t0 < WAIT


def test_pr08_waiter_woken_before_reap():
    # CVC_PYCVC_TEST_QUIESCE_DELAY_MS=200: run() sleeps 200 ms holding the GIL between its quiesce,
    # which answers the monitor's waiting call and wakes it, and its reap's lock. A monitor that
    # took the GIL back before letting the control mutex go would deadlock with run() there.
    times = []
    for i in range(20):
        os.environ["CVC_PYCVC_TEST_QUIESCE_DELAY_MS"] = "200"
        try:
            app = pycvc.make_app()
            ex = pycvc.Exec(app)
        finally:
            del os.environ["CVC_PYCVC_TEST_QUIESCE_DELAY_MS"]
        got = {}

        def monitor(pid):
            try:
                got["answer"] = ex.get_priority(pid)
            except RuntimeError as e:
                got["error"] = str(e)

        def last():
            pid = ex.scheduler().current_pid()
            got["steps"] = ex.scheduler().get_stats().total_steps
            t = threading.Thread(target=monitor, args=(pid,), name=f"pr08-waiter-{i}")
            got["thread"] = t
            t.start()
            time.sleep(0.3)  # the GIL released: the monitor's call is posted before the step ends
            return 1

        ex.register_fn("last", last)
        t0 = time.monotonic()
        assert ex.run("(last)") == "1"
        # Nothing stepped after (last), so nothing drained the posted call: it was still waiting
        # when run() left its loop, and run()'s quiesce answered it.
        assert ex.scheduler().get_stats().total_steps == got["steps"] + 1
        _join(got["thread"])
        times.append(time.monotonic() - t0)
        assert "error" in got or got["answer"] == _info(0)
        assert got.get("error", "Exec.run ended") == "Exec.run ended"
    print(f"  20 runs, run() and the waiter back within {max(times):.2f} s at most")
    assert max(times) < WAIT


# A sibling process with no env (builtins only) that runs a few hundred steps, then ends.
_SIBLING = "(begin (set i 0) (while (< i 300) (set i (+ i 1))) 1)"


def _program(end):
    # started() gives the monitor the pid; a few sleeps, so calls land while run() pumps; then the
    # program terminates, kills itself or raises.
    body = {
        "terminate": "1",
        "kill": "(kill (self-pid))",
        "raise": '(raise (ValueError "pr08"))',
    }[end]
    return f"(begin (started) (for i (range 5) (sleep 0)) {body})"


def _monitored_runs(ex, end, call, setup=None):
    """RUNS runs of _program(end) on `ex`, a monitor thread making `call(pid)` in a loop across
    each run's end. Returns the outcomes seen during the runs, those after them (once run() has
    returned, so the pid is reaped), and the errors the runs ended with."""
    during, after, errors = [], [], []
    box = {}

    def on_start():
        box["pid"] = ex.scheduler().current_pid()
        box["started"].set()
        return True

    ex.register_fn("started", on_start)
    t0 = time.monotonic()
    for i in range(RUNS):
        box["started"] = threading.Event()
        stop = threading.Event()

        def monitor():
            if not box["started"].wait(WAIT):
                return
            pid = box["pid"]
            while not stop.is_set():
                try:
                    during.append(call(pid))
                except RuntimeError as e:
                    during.append(str(e))
            after.append(call(pid))

        t = threading.Thread(target=monitor, name=f"pr08-{end}-{i}")
        t.start()
        if setup:
            setup()
        try:
            ex.run(_program(end))
        except Exception as e:  # noqa: BLE001 - a kill, a raise or an on_complete ends run()
            errors.append(str(e))
        stop.set()
        _join(t)
    print(f"  {RUNS} runs ({end}): {time.monotonic() - t0:.2f} s, {len(during)} calls")
    return during, after, errors


def test_pr08_stress_monitor_polls_get_priority():
    # A monitor polling get_priority while the program terminates, is killed and raises: every
    # call answered (a dict while the pid lives) or failed with "Exec.run ended", none left
    # waiting (the joins).
    for end in ("terminate", "kill", "raise"):
        app = pycvc.make_app()
        ex = pycvc.Exec(app)
        during, after, errors = _monitored_runs(ex, end, ex.get_priority)
        for out in during:
            assert out is None or out == "Exec.run ended" or out["base"] == 0, out
        assert after == [None] * len(after)  # once run() has returned, its pid is reaped
        assert len(errors) == (0 if end == "terminate" else RUNS), (end, errors[:3])


def test_pr08_stress_monitor_sets_and_gets_across_the_quiesce():
    # get_priority and set_priority in a loop across the quiesce, on run()'s normal path (the
    # program ends) and its catch path (a sibling's on_complete raises out of a step): no race
    # with run()'s reap, which is under the control mutex as the direct calls are (TSan), and a
    # reaped pid answered None / False.
    def both(pid):
        return (ex.set_priority(pid, 5), ex.get_priority(pid))

    def boom(_result):
        raise ValueError("pr08 on_complete")

    def sibling():
        # It runs beside the program, a step each in turn, and ends after the program's (started):
        # its on_complete raises then. (No env: builtins only.)
        opts = pycvc.execute_options()
        opts.on_complete = boom
        ex.scheduler().execute(_SIBLING, opts)

    for path, setup in (("normal", None), ("catch", sibling)):
        app = pycvc.make_app()
        ex = pycvc.Exec(app)
        during, after, errors = _monitored_runs(ex, "terminate", both, setup)
        # Applied and read back; or reaped between the two calls; or reaped before both.
        allowed = [(True, _info(5)), (True, None), (False, None)]
        for out in during:
            assert out in allowed, (path, out)
        assert after == [(False, None)] * len(after), path
        if path == "normal":
            assert errors == []
        else:  # every run took the catch path
            assert len(errors) == RUNS and all("pr08 on_complete" in e for e in errors), errors[:3]


def test_pr08_catch_path_raises_the_on_complete_error():
    # The catch path itself: a sibling's on_complete raising out of a step ends run() with that
    # error, after its quiesce and its reap (the GIL taken back first).
    app = pycvc.make_app()
    ex = pycvc.Exec(app)

    def boom(_result):
        raise ValueError("pr08 on_complete")

    opts = pycvc.execute_options()
    opts.on_complete = boom
    sibling = ex.scheduler().execute(_SIBLING, opts)
    pid_box = {}
    ex.register_fn("mark", lambda: pid_box.setdefault("pid", ex.scheduler().current_pid()))
    e = _raises(RuntimeError, ex.run, "(begin (mark) (for i (range 5) (sleep 0)) 1)")
    assert "pr08 on_complete" in str(e)
    assert ex.scheduler().get_process_info(sibling).status == pycvc.process_status.terminated
    assert ex.get_priority(pid_box["pid"]) is None  # reaped
    assert ex.run("(+ 1 2)") == "3"  # and the Exec runs again


def test_pr08_stress_kernel_calls_get_priority():
    # An async function's kernel (a pool thread) calling get_priority on its own Exec while the
    # program is killed and while it terminates: answered by run() between steps, or acting
    # directly during the quiesce (run() waits for the kernel then, the GIL released), no hang.
    for end in ("terminate", "kill"):
        app = pycvc.make_app()
        ex = pycvc.Exec(app)
        answers = []

        def kernel(pid):
            for _ in range(5):
                answers.append(ex.get_priority(pid))
            return 0

        ex.register_async_fn("k", kernel)
        body = "7" if end == "terminate" else "(kill (self-pid))"
        t0 = time.monotonic()
        for _ in range(RUNS):
            try:
                ex.run(f"(begin (k-async (self-pid)) (for i (range 3) (sleep 0)) {body})")
            except RuntimeError as e:
                assert end == "kill" and "kill" in str(e), str(e)
        elapsed = time.monotonic() - t0
        print(f"  {RUNS} runs ({end}): {elapsed:.2f} s, {len(answers) // 5} kernels ran")
        # A kernel the kill cancels before it starts never runs; one that runs makes its 5 calls.
        assert len(answers) % 5 == 0 and len(answers) > 0
        if end == "terminate":
            assert len(answers) == 5 * RUNS
        # run() waits for its kernels before it reaps: every answer is the live pid's.
        assert all(a == _info(0) for a in answers), [a for a in answers if a != _info(0)][:3]


def test_pr08_two_execs_call_each_other():
    # Two Execs on two threads whose registered functions call each other's get_priority: a call
    # from a thread running another Exec's run() raises at once, so neither waits for the other.
    app = pycvc.make_app()
    a, b = pycvc.Exec(app), pycvc.Exec(app)
    both_running = threading.Barrier(2, timeout=WAIT)
    errors = {}

    def caller(me, other, name):
        def call():
            both_running.wait()
            try:
                other.get_priority(1)
            except RuntimeError as e:
                errors[name] = str(e)
            both_running.wait()  # neither run() ends before both have called
            return True

        me.register_fn("call-other", call)

    caller(a, b, "a")
    caller(b, a, "b")
    results = {}
    tb = threading.Thread(target=lambda: results.update(b=b.run("(call-other)")), name="pr08-b")
    tb.start()
    results["a"] = a.run("(call-other)")
    _join(tb)
    assert results == {"a": "#t", "b": "#t"}
    assert errors == {
        "a": "Exec.run is active on another thread",
        "b": "Exec.run is active on another thread",
    }


def test_pr08_run_reentered_during_the_quiesce():
    # A kernel, and another thread it starts, call run() on the same Exec while its run() waits
    # for that kernel (the quiesce): today's re-entrancy RuntimeError at once, and the first run()
    # returns its result.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    ended = threading.Event()
    errors = []

    def again(tag):
        try:
            ex.run("1")
        except RuntimeError as e:
            errors.append((tag, str(e)))

    def kernel():
        ended.wait(WAIT)
        time.sleep(0.05)  # the program has ended: run() waits for this kernel
        again("kernel")
        t = threading.Thread(target=again, args=("thread",), name="pr08-reenter")
        t.start()
        t.join(WAIT)
        return 0

    ex.register_async_fn("k", kernel)
    ex.register_fn("end", lambda: ended.set() or True)
    t0 = time.monotonic()
    assert ex.run("(begin (k-async) (end) 7)") == "7"
    assert time.monotonic() - t0 < WAIT
    assert sorted(tag for tag, _ in errors) == ["kernel", "thread"]
    assert all("re-entrant" in msg for _, msg in errors), errors
    assert ex.run("(+ 1 1)") == "2"


# ── L2: thread classes (PR-9) ───────────────────────────────────────────


def test_pr09_set_thread_class():
    # Reported, never raised: on a thread of its own, since on Linux an unprivileged thread can't
    # raise its nice value back.
    tc = pycvc.thread_class
    assert [m.name for m in tc] == ["background", "utility", "normal"]
    out = {}

    def classes():
        out["results"] = [pycvc.set_thread_class(c) for c in (tc.background, tc.utility, tc.normal)]

    t = threading.Thread(target=classes, name="pr09")
    t.start()
    _join(t)
    results = out["results"]
    print(f"  {sys.platform}: background, utility, normal -> {results}")
    assert all(isinstance(r, bool) for r in results)
    if sys.platform == "win32":
        assert results == [True, True, True]
    elif sys.platform.startswith("linux"):
        assert results[0] is True  # a demotion needs no privilege
    _raises(TypeError, pycvc.set_thread_class, 0)  # a pycvc.thread_class member, as for any enum


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc state_exec locks tests: OK")
