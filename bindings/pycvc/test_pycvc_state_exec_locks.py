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

L3: Execs on several Python threads share the app's lock manager (design 9.2):
cross-thread sections and grants (LT-1, LT-3, LT-10's smoke), run()'s idle rule
for a lock wait (LT-7, LT-8, LT-8b), pids hidden across Execs (LT-11), a
deadlock across Execs (LD-5), and Exec.lock_query from any thread (PR-8).
CVC_LOCKS_SANITIZER=1 (the sanitizer jobs) runs the smaller sizes.
"""

import asyncio
import faulthandler
import gc
import json
import math
import os
import re
import sys
import threading
import time

# Every lock manager operation checks the manager's invariants (gap G-38); the
# library reads this once, when the first manager is made. CVC_STATE_EXEC_CHECK=0
# in the environment runs without the checks.
os.environ.setdefault("CVC_STATE_EXEC_CHECK", "1")

import pycvc  # noqa: E402
from pycvc import eval_phase, process_status  # noqa: E402

_ENDED = (process_status.terminated, process_status.killed, process_status.failed)

RUNS = int(os.environ.get("CVC_PYCVC_LOCKS_RUNS", "500"))
# A case's outcome must come within this many seconds (a hang fails it); the times are printed.
WAIT = 10.0
# The sanitizer jobs' sizes (L1 section 3, "Sizes, by lane").
SANITIZER = os.environ.get("CVC_LOCKS_SANITIZER") == "1"
# The ceiling for a case that runs 10^4-10^5 sections across threads (its time is printed).
LONG = 600.0


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
        # The interleaving was forced: run() spent the 0.3 s sleep and the 200 ms delay (a delay
        # the library never read, as from another C runtime's environment, would force nothing).
        ran = time.monotonic() - t0
        assert ran >= 0.45, f"run() took {ran:.3f} s: was the quiesce delay applied?"
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
    # program terminates, kills itself or raises, or waits for a sibling to end (sibling-ended,
    # which the test registers).
    body = {
        "terminate": "1",
        "kill": "(kill (self-pid))",
        "raise": '(raise (ValueError "pr08"))',
        "sibling": "(begin (while (not (sibling-ended)) (sleep 0)) 1)",
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

    ended = threading.Event()
    box = {}

    def boom(_result):
        ended.set()
        raise ValueError("pr08 on_complete")

    def sibling():
        # It runs beside the program, a step each in turn, and the program waits for it to end
        # (bounded): its on_complete raises then, in this run. A program that ended first left its
        # sibling to end in a later run, as it did under TSan. (No env: builtins only.)
        ended.clear()
        box["deadline"] = time.monotonic() + WAIT
        opts = pycvc.execute_options()
        opts.on_complete = boom
        ex.scheduler().execute(_SIBLING, opts)

    for path, end, setup in (("normal", "terminate", None), ("catch", "sibling", sibling)):
        app = pycvc.make_app()
        ex = pycvc.Exec(app)
        ex.register_fn(
            "sibling-ended", lambda: ended.is_set() or time.monotonic() > box["deadline"]
        )
        during, after, errors = _monitored_runs(ex, end, both, setup)
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


def test_pr08_reap_frees_python_after_the_mutex():
    # The process run() reaps can hold the last reference to a Python object: on the catch path,
    # the exception the program failed with, whose traceback keeps the raising frame and its
    # locals. Its finalizer here calls the Exec on the run() thread, then releases the GIL while
    # another thread calls run(). Freed under the control mutex, the call relocks the mutex, and
    # run() waits for the GIL the other thread holds while that thread waits for the mutex. A hang
    # fails the case: faulthandler prints every thread's stack and exits.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    freed = threading.Event()
    reentered = threading.Event()
    seen = {}

    class Held:
        def __del__(self):
            seen["get_priority"] = ex.get_priority(seen["pid"])
            freed.set()
            reentered.wait(WAIT)  # the GIL released until the other thread's run() returns

    def boom():
        seen["pid"] = ex.scheduler().current_pid()
        held = Held()  # noqa: F841 - a local of the raising frame, kept by its traceback
        raise ValueError("pr08 boom")

    def sibling_done(_result):
        raise ValueError("pr08 on_complete")

    opts = pycvc.execute_options()
    opts.on_complete = sibling_done
    ex.scheduler().execute(_SIBLING, opts)
    ex.register_fn("boom", boom)

    def reenter():
        try:
            if freed.wait(WAIT):
                seen["reentry"] = str(_raises(RuntimeError, ex.run, "1"))
        finally:
            reentered.set()

    t = threading.Thread(target=reenter, name="pr08-reenter")
    t.start()
    faulthandler.dump_traceback_later(3 * WAIT, exit=True)
    try:
        e = _raises(RuntimeError, ex.run, "(boom)")
    finally:
        faulthandler.cancel_dump_traceback_later()
    _join(t)
    assert "pr08 on_complete" in str(e)  # the catch path: nothing but the process held it
    assert freed.is_set()
    assert seen["get_priority"] is None  # reaped by then
    assert "re-entrant" in seen["reentry"]
    assert ex.run("(+ 1 2)") == "3"


def test_pr08_reap_runs_no_python_under_the_mutex():
    # The reap cancels the program's async calls still in flight, and their charge reads the step
    # rate, which a scheduler with an id publishes to the tree, where a state_observer's
    # on_changed runs Python on the publishing thread. Here the call ends after run() left its
    # loop, having changed the rate setting, so the reap's read publishes the new rate, and the
    # observer calls the Exec. Under the control mutex that call relocks it. A hang fails the case
    # through faulthandler.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    ex.scheduler().set_id("pr08")
    published = "state_exec.schedulers.pr08.effective.async_steps_per_cpu_ms"
    seen = []

    class Rate(pycvc.state_observer):
        def on_changed(self, path):
            if path == published and pycvc.state_get(app, path) == "123":
                # Whether the program had ended: then this is the reap's publish, not a charge's
                # inside run()'s loop (a run() slower than the kernel's second would leave the
                # loop after it).
                ended = all(p.status in _ENDED for p in ex.scheduler().list_processes())
                seen.append((ex.get_priority(0), ended))

    def late():
        time.sleep(1.0)  # run() has left its loop and waits for this call (wait_idle)
        pycvc.state_set(app, "state_exec.schedulers.pr08.limits.async_steps_per_cpu_ms", "123")
        return 1

    ex.register_async_fn("late", late)
    obs = Rate()
    obs.watch(app)
    faulthandler.dump_traceback_later(3 * WAIT, exit=True)
    try:
        assert ex.run("(begin (late-async) 1)") == "1"  # its future never awaited
    finally:
        faulthandler.cancel_dump_traceback_later()
        obs.unwatch()
    # The reap's publish ran the observer, after the program ended, and its call was answered.
    assert seen == [(None, True)]


def test_pr08_idle_calls_beside_scheduler_calls():
    # With no run() active a call acts directly, holding the GIL and the control mutex, so it
    # never touches the process table while this thread changes it through ex.scheduler(), which
    # holds only the GIL. A call acting without the GIL is a race on the table: reported in C0's
    # TSan job, a crash at worst elsewhere.
    app = pycvc.make_app()
    ex = pycvc.Exec(app, policy="priority_rr")
    pid = ex.scheduler().execute("(sleep 100)")
    stop = threading.Event()
    calls = {"n": 0, "wrong": []}

    def monitor():
        while not stop.is_set():
            applied = ex.set_priority(pid, 3)
            info = ex.get_priority(pid)
            if applied is not True or info is None or info["base"] != 3:
                calls["wrong"].append((applied, info))
            calls["n"] += 1

    t = threading.Thread(target=monitor, name="pr08-idle-monitor")
    t.start()
    try:
        n, until = 0, time.monotonic() + 0.5
        while n < RUNS or time.monotonic() < until:
            p = ex.scheduler().execute("(sleep 100)")
            assert ex.scheduler().kill(p)
            assert ex.scheduler().reap(p)
            n += 1
    finally:
        stop.set()
        _join(t)
    print(f"  {calls['n']} calls beside {n} submits, kills and reaps")
    assert calls["wrong"] == []
    assert calls["n"] > 0


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


# ── L3: Execs on several threads (design 9.2) ───────────────────────────


def _in_thread(name, fn, *args, start=None):
    """Start fn(*args) on a thread, once every party has reached `start` (a threading.Barrier), if
    given. The dict it returns gets "value" or "error", and "t" (the time it ended)."""
    out = {}

    def body():
        try:
            if start is not None:
                start.wait(WAIT)
            out["value"] = fn(*args)
        except BaseException as e:  # noqa: BLE001 - the case reads it
            out["error"] = e
        out["t"] = time.monotonic()

    t = threading.Thread(target=body, name=name)
    t.start()
    return t, out


def _wait_until(cond, seconds=WAIT, pump=None):
    """Poll cond() (calling pump() between polls, if given) until it holds; False after seconds."""
    end = time.monotonic() + seconds
    while not cond():
        if time.monotonic() > end:
            return False
        if pump is not None:
            pump()
        time.sleep(0.001)
    return True


def _holders(ex, key):
    """The pids ex sees holding `key` (its own processes only) and the hold count."""
    held = [h for h in ex.lock_query(key)["held"] if h["key"] == key]
    return (held[0]["pids"], held[0]["count"]) if held else ([], 0)


def _spawn_holder(ex, key, channel):
    """ex spawns a child that takes `key` and parks on msg-recv `channel`. Returns its pid once it
    holds the key; ex's run() has returned, so nothing pumps ex."""
    pid = int(ex.run(
        f'(spawn "(with-lock \\"{key}\\" (msg-recv \\"{channel}\\"))")'))
    # The child steps while a run() pumps the Exec.
    assert _wait_until(lambda: _holders(ex, key.lstrip("/"))[0] == [pid],
                       pump=lambda: ex.run("(sleep 0.001)")), key
    return pid


def _queued(ex, key):
    return bool(ex.lock_query(key)["waiting"])


def test_l3_exec_needs_a_shared_manager():
    # An Exec may run() on any Python thread, so its home attaches as one pumped on a thread of its
    # own (design 8.3): a single-mode app manager refuses it, naming the setting, and so it does a
    # scheduler's set_lock_manager(app). (auto, the default, is shared once pycvc is imported.)
    app = pycvc.make_app()
    pycvc.state_set(app, "state_exec.locks.threading", "single")
    e = _raises(RuntimeError, pycvc.Exec, app)
    assert "state_exec.locks.threading" in str(e), e
    e = _raises(RuntimeError, pycvc.scheduler().set_lock_manager, app)
    assert "state_exec.locks.threading" in str(e), e
    app = pycvc.make_app()
    pycvc.state_set(app, "state_exec.locks.threading", "auto")
    assert pycvc.Exec(app).run('(with-lock "k" (+ 1 2))') == "3"


def test_l3_set_lock_manager_attaches_only_a_first_home():
    # scheduler.set_lock_manager(app) attaches a scheduler's first lock home only: a home replaced
    # would be freed under what uses it. An Exec's run() and lock_query use their scheduler's, so a
    # swap from inside run() raises (it used to free the home run() kept), and the Exec runs on.
    app = pycvc.make_app()
    ex = pycvc.Exec(app, idle_timeout=0.2)
    ex.register_fn("swap", lambda: ex.scheduler().set_lock_manager(app) or True)
    e = _raises(RuntimeError, ex.run, '(begin (swap) (msg-recv "never"))')
    assert "already has its lock home" in str(e), e
    assert ex.run('(with-lock "k" (+ 1 2))') == "3"
    assert ex.lock_query("") == {"held": [], "waiting": []}
    # A fresh scheduler attaches (LT-7 steps one), and then has its home; so does one whose first
    # with-lock made it a private one.
    s, env = _app_scheduler(app)
    pid = s.execute('(with-lock "k" 1)', _env_opts(env))
    s.run()
    assert s.get_result(pid) == 1
    _raises(RuntimeError, s.set_lock_manager, app)
    t = pycvc.scheduler()
    ctx = pycvc.intrinsics_context()
    ctx.sched = t
    ctx.root = pycvc.State.instance(app)
    env = pycvc.builtins.make_default_environment()
    pycvc.register_intrinsics(env, ctx)
    pid = t.execute('(with-lock "k" 2)', _env_opts(env))
    t.run()
    assert t.get_result(pid) == 2
    e = _raises(RuntimeError, t.set_lock_manager, app)
    assert "already has its lock home" in str(e), e
    for sched in (s, t):
        for p in [i.pid for i in sched.list_processes()]:
            assert sched.reap(p)  # their environments hold the context, which holds the scheduler


def test_lt01_two_execs_hot_key():
    # LT-1: two Execs on two Python threads, one hot key, each section a read-modify-write of a
    # tree value, which isn't atomic without the lock: the count exact.
    n = 10**4 if SANITIZER else 10**5
    app = pycvc.make_app()
    pycvc.state_set(app, "lt01.n", "0")
    prog = (f'(for i (range {n}) (with-lock "lt01.k" '
            f'(state-set "lt01.n" (+ (int (state-get "lt01.n")) 1))))')
    execs = [pycvc.Exec(app) for _ in range(2)]
    start = threading.Barrier(len(execs))
    t0 = time.monotonic()
    runs = [_in_thread(f"lt01-{i}", ex.run, prog, start=start) for i, ex in enumerate(execs)]
    for t, out in runs:
        t.join(LONG)
        assert not t.is_alive(), f"{t.name} did not finish within {LONG} s"
        assert "error" not in out, out["error"]
    print(f"  2 x {n} sections on one key: {time.monotonic() - t0:.2f} s")
    assert int(pycvc.state_get(app, "lt01.n")) == 2 * n
    assert execs[0].lock_query("") == {"held": [], "waiting": []}


def test_lt03_exec_destroyed():
    # LT-3: an Exec's holder killed while another Exec waits: the waiter granted. An Exec destroyed
    # with a wake in its port: its processes hold nothing after, and the manager (checked after
    # every operation) goes on.
    app = pycvc.make_app()
    a, b = pycvc.Exec(app), pycvc.Exec(app)
    holder = _spawn_holder(a, "lt03.k", "lt03.never")
    tb, ob = _in_thread("lt03-b", b.run, '(with-lock "lt03.k" "b")')
    assert _wait_until(lambda: _queued(a, "lt03.k"))
    assert a.scheduler().kill(holder)  # no run() of A is active: A's home acts here
    _join(tb)
    assert ob.get("value") == "b", ob

    # C's waiter (a child its run() spawned) is granted while nobody pumps C, so the grant's wake
    # waits in C's port; then C goes.
    holder = _spawn_holder(a, "lt03.k2", "lt03.go")
    c = pycvc.Exec(app)
    waiter = int(c.run('(spawn "(with-lock \\"lt03.k2\\" 1)")'))
    assert _wait_until(lambda: _queued(a, "lt03.k2"), pump=lambda: c.run("(sleep 0.001)"))
    assert c.scheduler().get_process_info(waiter).waiting_on == "lock /lt03.k2"
    # A's holder gets its message and releases, in A's run().
    a.run('(msg-send "lt03.go" "go")')
    assert _wait_until(
        lambda: a.scheduler().get_process_info(holder).status in _ENDED,
        pump=lambda: a.run("(sleep 0.001)"))
    assert _holders(a, "lt03.k2") == ([], 1)  # C's now, by the manager; C's pid isn't A's to see
    assert not _queued(a, "lt03.k2")
    assert c.scheduler().get_process_info(waiter).status == process_status.waiting  # not drained
    del c
    gc.collect()
    assert a.lock_query("") == {"held": [], "waiting": []}
    assert a.run('(with-lock "lt03.k2" 7)') == "7"


def _app_scheduler(app):
    """A sync scheduler over app's tree, on app's lock manager, and an environment of intrinsics
    bound to it."""
    root = pycvc.State.instance(app)
    s = pycvc.scheduler()
    s.set_watch_root(root)
    s.set_lock_manager(app)
    ctx = pycvc.intrinsics_context()
    ctx.sched = s
    ctx.root = root
    env = pycvc.builtins.make_default_environment()
    pycvc.register_intrinsics(env, ctx)
    return s, env


def _env_opts(env):
    o = pycvc.execute_options()
    o.env = env
    return o


def test_lt07_progress_keeps_waiter_alive():
    # LT-7: a waiter (idle_timeout 0.5) behind a holder whose home keeps making progress for longer
    # than the limit: granted when the holder releases, never killed.
    app = pycvc.make_app()
    b = pycvc.Exec(app, idle_timeout=0.5)

    # (1) Another Exec's holder across a 1.5 s await, its run() pumping.
    a = pycvc.Exec(app, idle_timeout=0.5)
    holds = threading.Event()
    released = {}
    a.register_fn("holds", lambda: holds.set() or True)
    a.register_async_fn("slow", lambda s: time.sleep(s) or True)
    a.register_fn("releasing", lambda: released.setdefault("a", time.monotonic()) > 0)
    ta, oa = _in_thread("lt07-a", a.run,
                        '(with-lock "lt07.a" (begin (holds) (slow 1.5) (releasing) "a"))')
    assert holds.wait(WAIT)
    t0 = time.monotonic()
    got = b.run('(with-lock "lt07.a" "b")')
    granted = time.monotonic()
    _join(ta)
    print(f"  behind an Exec's 1.5 s await: granted after {granted - t0:.2f} s")
    assert got == "b" and oa.get("value") == "a", oa
    assert granted >= released["a"] and granted - t0 < WAIT

    # (2) A sync scheduler's holder that steps every 0.1 s for 2 s, in one run() on its thread.
    s, env = _app_scheduler(app)
    pycvc.builtins.register_fn(env, "releasing",
                               lambda k: released.setdefault(k, time.monotonic()) > 0)
    holder = s.execute(
        '(with-lock "lt07.s" (begin (for i (range 20) (sleep 0.1)) (releasing "s") "s"))',
        _env_opts(env))
    assert _wait_until(lambda: s.get_process_info(holder).locks_held == 1, pump=s.step)
    ts, os_ = _in_thread("lt07-s", s.run)
    t0 = time.monotonic()
    got = b.run('(with-lock "lt07.s" "b")')
    granted = time.monotonic()
    _join(ts)
    print(f"  behind a sync run() stepping every 0.1 s: granted after {granted - t0:.2f} s")
    assert got == "b" and "error" not in os_, os_
    assert s.get_result(holder) == "s"
    assert granted >= released["s"] and granted - t0 < WAIT

    # (3) The same scheduler pumped only by a step() every 0.1 s, for 2 s before the holder may
    # leave its section.
    go = threading.Event()
    pycvc.builtins.register_fn(env, "go?", lambda: go.is_set())
    holder = s.execute(
        '(with-lock "lt07.t" (begin (while (not (go?)) (sleep 0)) (releasing "t") "t"))',
        _env_opts(env))
    assert _wait_until(lambda: s.get_process_info(holder).locks_held == 1, pump=s.step)

    def pump():
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            s.step()
            time.sleep(0.1)
        go.set()
        assert _wait_until(lambda: s.get_process_info(holder).status in _ENDED, pump=s.step)

    tp, op = _in_thread("lt07-step", pump)
    t0 = time.monotonic()
    got = b.run('(with-lock "lt07.t" "b")')
    granted = time.monotonic()
    _join(tp)
    print(f"  behind a step() every 0.1 s: granted after {granted - t0:.2f} s")
    assert got == "b" and "error" not in op, op
    assert granted >= released["t"] and granted - t0 < WAIT
    for pid in [p.pid for p in s.list_processes()]:
        assert s.reap(pid)  # their environments hold the context, which holds the scheduler


def test_lt07b_spawned_waiter_counts():
    # The idle rule counts a lock wait of any of the Exec's processes, not only run()'s own
    # (design 9.2: "a lock wait"). A's program waits on msg-recv for a child that waits for a key B
    # holds for 2 s while it keeps stepping: granted after B releases, never killed. Behind a B
    # nobody pumps, the same is killed at the limit, naming the child and its key.
    app = pycvc.make_app()
    a = pycvc.Exec(app, idle_timeout=0.5)
    b = pycvc.Exec(app)
    holds = threading.Event()
    released = {}
    b.register_fn("holds", lambda: holds.set() or True)
    b.register_fn("releasing", lambda: released.setdefault("t", time.monotonic()) > 0)

    def child_waits(key, channel):
        return (f'(begin (spawn "(with-lock \\"{key}\\" (msg-send \\"{channel}\\" \\"ok\\"))") '
                f'(msg-recv "{channel}"))')

    tb, ob = _in_thread("lt07b-b", b.run, '(with-lock "lt07b.k" (begin (holds) '
                        '(for i (range 20) (sleep 0.1)) (releasing)))')
    assert holds.wait(WAIT)
    t0 = time.monotonic()
    got = a.run(child_waits("lt07b.k", "lt07b.done"))
    done = time.monotonic()
    _join(tb)
    print(f"  a spawned waiter behind B's 2 s section: granted after {done - t0:.2f} s")
    assert json.loads(got)["payload"] == "ok" and "error" not in ob, (got, ob)  # the message
    assert done >= released["t"] and done - t0 < WAIT

    _spawn_holder(b, "lt07b.k2", "lt07b.never")
    t0 = time.monotonic()
    e = _raises(RuntimeError, a.run, child_waits("lt07b.k2", "lt07b.done2"))
    print(f"  behind an unpumped Exec: killed after {time.monotonic() - t0:.2f} s: {e}")
    assert re.search(r"parked with no pending work \(pid \d+, pid \d+ waiting for lock lt07b\.k2\)",
                     str(e)), e
    assert time.monotonic() - t0 < WAIT


def test_lt08_idle_limit():
    # LT-8: the idle rule is per home. B (idle_timeout 0.5) waits behind A's holder.
    idle = 0.5
    app = pycvc.make_app()
    a = pycvc.Exec(app)
    b = pycvc.Exec(app, idle_timeout=idle)

    # (1) A's run() has returned and nothing pumps A: B is killed at the limit, naming the key.
    _spawn_holder(a, "lt08.k1", "lt08.never")
    t0 = time.monotonic()
    e = _raises(RuntimeError, b.run, '(with-lock "lt08.k1" 1)')
    print(f"  behind an unpumped Exec: killed after {time.monotonic() - t0:.2f} s: {e}")
    assert "parked with no pending work" in str(e) and "waiting for lock lt08.k1)" in str(e)
    assert time.monotonic() - t0 < WAIT

    # (2) A pumped, its program stepping every 0.05 s for 1 s before it sends the holder's
    # message: B granted when the holder releases.
    _spawn_holder(a, "lt08.k2", "lt08.go")
    tb, ob = _in_thread("lt08-b2", b.run, '(with-lock "lt08.k2" "b")')
    assert _wait_until(lambda: _queued(a, "lt08.k2"))
    # (The last sleep lets the holder step to its release before this run() returns.)
    a.run('(begin (for i (range 20) (sleep 0.05)) (msg-send "lt08.go" "go") (sleep 0.05))')
    _join(tb)
    assert ob.get("value") == "b", ob

    # (3) A pumped while its holder stays parked on msg-recv, a sibling (A's program) stepping
    # every 0.05 s for 2 s: B alive while A steps, then killed at the limit, which runs from A's
    # last progress: no earlier than idle_timeout after A's last step began.
    _spawn_holder(a, "lt08.k3", "lt08.never")
    stepped = {}
    a.register_fn("stepped", lambda: stepped.setdefault("t", time.monotonic()) > 0)
    tb, ob = _in_thread("lt08-b3", b.run, '(with-lock "lt08.k3" 1)')
    assert _wait_until(lambda: _queued(a, "lt08.k3"))
    a.run("(begin (for i (range 40) (sleep 0.05)) (stepped))")
    _join(tb)
    e = ob.get("error")
    gap = ob["t"] - stepped["t"]
    print(f"  killed {gap:.2f} s after A's last step (idle_timeout {idle} s): {e}")
    assert isinstance(e, RuntimeError) and "waiting for lock lt08.k3)" in str(e), ob
    assert idle <= gap < WAIT


def test_lt08c_grant_posted_before_the_idle_test():
    # A grant posted to the Exec's lock port after run()'s drain, in the pass whose idle test finds
    # the limit passed, is work pending, which the next pass applies: not a kill. idle_timeout=0
    # makes the limit pass at the first pass with nothing else pending. B's own coroutine, which
    # runs between B's steps and that idle test, kills A's holder after 50 ms, and B's grant is
    # posted then.
    app = pycvc.make_app()
    a = pycvc.Exec(app)
    holder = _spawn_holder(a, "lt08c.k", "lt08c.never")
    b = pycvc.Exec(app, idle_timeout=0.0)

    async def release():
        await asyncio.sleep(0.05)
        return a.scheduler().kill(holder)

    b.register_async_coro("release", release)
    assert b.run('(begin (spawn "(release)") (with-lock "lt08c.k" "b"))') == "b"
    assert a.scheduler().get_process_info(holder).status == process_status.killed


def test_lt08b_same_exec_holder_parked():
    # LT-8b: the holder is the waiter's own Exec's, parked on msg-recv with nothing else pending
    # there: this run() pumping it is no progress. Killed at the limit, naming the key.
    app = pycvc.make_app()
    ex = pycvc.Exec(app, idle_timeout=0.5)
    t0 = time.monotonic()
    e = _raises(RuntimeError, ex.run,
                '(begin (spawn "(with-lock \\"lt08b.k\\" (msg-recv \\"never\\"))") (sleep 0.1) '
                '(with-lock "lt08b.k" 1))')
    elapsed = time.monotonic() - t0
    print(f"  killed after {elapsed:.2f} s: {e}")
    assert "parked with no pending work" in str(e) and "waiting for lock lt08b.k)" in str(e)
    assert elapsed < WAIT
    assert ex.run("(+ 1 2)") == "3"  # run() returned, and the Exec runs again


def test_lt10_eight_execs_smoke():
    # LT-10's smoke: 8 Execs on 8 Python threads, sections over 4 keys, each a read-modify-write
    # of its key's counter: every counter exact, every run() returns, and nothing is held or
    # waiting at the end.
    keys, nexec = 4, 8
    rounds = 63 if SANITIZER else 313  # 8 x 4 x rounds: about 2*10^3, or 10^4, sections
    app = pycvc.make_app()
    for k in range(keys):
        pycvc.state_set(app, f"lt10.c{k}", "0")

    def section(k):
        return (f'(with-lock "lt10.k{k}" '
                f'(state-set "lt10.c{k}" (+ (int (state-get "lt10.c{k}")) 1)))')

    execs = [pycvc.Exec(app) for _ in range(nexec)]
    start = threading.Barrier(nexec)  # every run() starts together
    t0 = time.monotonic()
    runs = []
    for i, ex in enumerate(execs):
        body = " ".join(section((i + j) % keys) for j in range(keys))  # each starts elsewhere
        prog = f"(for i (range {rounds}) (begin {body}))"
        runs.append(_in_thread(f"lt10-{i}", ex.run, prog, start=start))
    for t, out in runs:
        t.join(LONG)
        assert not t.is_alive(), f"{t.name} did not finish within {LONG} s"
        assert "error" not in out, out["error"]
    print(f"  {nexec * keys * rounds} sections: {time.monotonic() - t0:.2f} s")
    for k in range(keys):
        assert int(pycvc.state_get(app, f"lt10.c{k}")) == nexec * rounds, k
    assert execs[0].lock_query("") == {"held": [], "waiting": []}


def _cycle_across_execs(app, tag):
    """A deadlock between two Execs on two threads (design 6.1): A's program takes k1, then asks
    for k2; B's takes k2, then asks for k1 once A's request is queued, closing the cycle. Returns
    A's run() outcome, B's error, and B's pid."""
    a, b = pycvc.Exec(app), pycvc.Exec(app)
    k1, k2 = f"{tag}.k1", f"{tag}.k2"
    a_holds, b_holds = threading.Event(), threading.Event()
    box = {}

    def a_ready():
        a_holds.set()
        return b_holds.wait(WAIT)  # A asks for k2 once B holds it

    def b_ready():
        box["pid"] = b.scheduler().current_pid()
        b_holds.set()
        # A's request is queued: A's own view shows it (lock_query never waits for a run()).
        return _wait_until(lambda: _queued(a, k2))

    a.register_fn("ready", a_ready)
    b.register_fn("ready", b_ready)
    ta, oa = _in_thread(f"{tag}-a", a.run,
                        f'(with-lock "{k1}" (begin (ready) (with-lock "{k2}" "a")))')
    assert a_holds.wait(WAIT)
    t0 = time.monotonic()
    e = _raises(pycvc.dsl_error, b.run,
                f'(with-lock "{k2}" (begin (ready) (with-lock "{k1}" "b")))')
    _join(ta)
    print(f"  {tag}: B raised after {time.monotonic() - t0:.2f} s: {e}; A: {oa}")
    return oa, e, box["pid"]


def test_ld05_cycle_across_execs():
    # LD-5: the request that closes a cycle across two Execs raises DeadlockError when it blocks;
    # its section unwinds, releasing k2, and A's request is granted.
    app = pycvc.make_app()
    oa, e, _ = _cycle_across_execs(app, "ld05")
    assert e.dsl_class == "DeadlockError", e
    assert oa.get("value") == "a", oa


def test_lt11_foreign_pids_hidden():
    # LT-11: pids repeat across Execs, so another Exec's are left out of what a program or
    # Exec.lock_query sees, while its holds are counted.
    app = pycvc.make_app()
    a, b = pycvc.Exec(app), pycvc.Exec(app)
    assert _spawn_holder(a, "/lt11.k", "lt11.never") == 2  # A's pid 2 holds X "/lt11.k"
    assert b.run("1") == "1" and b.run("2") == "2"  # B's next program is its pid 3
    seen = []
    b.register_fn("see", lambda v: seen.append(v) or True)
    b.run('(see (list (self-pid) (lock-query "/lt11.k")))')
    pid, q = seen[0]
    assert pid == 3
    assert q["waiting"] == [] and len(q["held"]) == 1, q
    h = q["held"][0]
    assert (h["key"], h["mode"], h["count"], h["pids"]) == ("lt11.k", "exclusive", 1, []), h
    assert _holders(b, "lt11.k") == ([], 1)
    assert _holders(a, "lt11.k") == ([2], 1)

    # A cycle closed by B: its DeadlockError's cycle shows B's own pid, and A's as nil.
    _, e, b_pid = _cycle_across_execs(app, "lt11")
    assert e.dsl_class == "DeadlockError", e
    cycle = e.value["cycle"]
    assert {c["pid"] for c in cycle} == {None, b_pid}, cycle
    assert {c["key"] for c in cycle} == {"lt11.k1", "lt11.k2"}, cycle


def test_pr08_lock_query_answered_never_waiting():
    # PR-8 (L3): lock_query reads the app's manager directly. Answered from another thread on an
    # idle Exec, and while run() is inside a step (a call that waited for run() to answer it
    # between steps would not return until the step ended), and from a thread running another
    # Exec's run() (where get_priority raises rather than wait).
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    holder = _spawn_holder(ex, "pr08q.k", "pr08q.never")
    t, out = _in_thread("pr08q-idle", ex.lock_query, "pr08q")
    _join(t)
    assert [(h["key"], h["pids"]) for h in out["value"]["held"]] == [("pr08q.k", [holder])], out

    seen = {}

    def inside():
        # On the run() thread, inside a step: another thread asks.
        t, out = _in_thread("pr08q-pumping", ex.lock_query, "pr08q.inside")
        t.join(WAIT)
        seen["alive"] = t.is_alive()
        seen["out"] = out
        seen["pid"] = ex.scheduler().current_pid()
        return True

    ex.register_fn("inside", inside)
    assert ex.run('(with-lock "pr08q.inside" (inside))') == "#t"
    assert not seen["alive"], "lock_query waited for run()"
    held = seen["out"]["value"]["held"]
    assert [(h["key"], h["pids"], h["count"]) for h in held] == \
        [("pr08q.inside", [seen["pid"]], 1)], seen

    other = pycvc.Exec(app)
    other.register_fn("ask", lambda: _holders(ex, "pr08q.k") == ([holder], 1))
    assert other.run("(ask)") == "#t"


def test_pr08_lock_query_stress_monitor():
    # PR-8 (L3): a monitor polling lock_query while the program terminates, is killed and raises,
    # inside its section: every answer whole, no hang (faulthandler dumps every stack and exits).
    # Each run's section waits for one of the monitor's answers, then ends while it polls on.
    runs = 10**3 if SANITIZER else 10**4
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    ends = ("1", "(kill (self-pid))", '(raise (ValueError "pr08q"))')
    stop = threading.Event()
    seen = {"answers": 0, "held": 0, "wrong": []}

    def monitor():
        while not stop.is_set():
            q = ex.lock_query("")
            seen["answers"] += 1
            if q["waiting"]:
                seen["wrong"].append(q)
            for h in q["held"]:
                seen["held"] += 1
                if (h["key"], h["mode"], h["count"]) != ("pr08q.s", "exclusive", 1) or \
                        len(h["pids"]) > 1:
                    seen["wrong"].append(q)
            time.sleep(0)  # lets the runs have the GIL between polls

    def polled_after(n):
        return seen["answers"] > n or time.monotonic() > seen["deadline"]

    ex.register_fn("answers", lambda: seen["answers"])
    ex.register_fn("polled-after?", polled_after)
    t = threading.Thread(target=monitor, name="pr08q-monitor")
    switch = sys.getswitchinterval()
    sys.setswitchinterval(1e-4)  # the GIL passes between the two threads every few steps
    t.start()
    faulthandler.dump_traceback_later(LONG, exit=True)
    errors = 0
    t0 = time.monotonic()
    try:
        for i in range(runs):
            end = ends[i % len(ends)]
            seen["deadline"] = time.monotonic() + WAIT
            try:
                ex.run('(with-lock "pr08q.s" (begin (set n (answers)) '
                       f'(while (not (polled-after? n)) (sleep 0)) {end}))')
            except RuntimeError:  # the kill, and the raise (a pycvc.dsl_error)
                errors += 1
    finally:
        stop.set()
        t.join(WAIT)
        faulthandler.cancel_dump_traceback_later()
        sys.setswitchinterval(switch)
    print(f"  {runs} runs: {time.monotonic() - t0:.2f} s, {seen['answers']} answers, "
          f"{seen['held']} holds seen")
    assert not t.is_alive()
    assert seen["answers"] >= runs and seen["held"] > 0
    assert seen["wrong"] == [], seen["wrong"][:3]
    assert errors == runs - len(range(0, runs, len(ends)))
    assert ex.lock_query("") == {"held": [], "waiting": []}


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc state_exec locks tests: OK")
