"""Long-running native calls from Python (docs/STATE_EXEC_EXCEPTIONS.md section 3, the AS cases of
section 6.6).

pycvc.builtins.register_async_fn(env, host, name, fn, ...) makes a Python callable an async
function: the calling process parks while fn runs on the host's executor, then resumes with its
value or raises its exception at the call. Exec.register_async_fn is built on it, with
Exec.async_host(). A call is charged to the process that made it: its run time, its CPU time and
steps at limits.async_steps_per_cpu_ms. The C++ cases are src/cvc/tests/state_exec_async_calls_test.cpp;
these repeat them from Python, with Python kernels, and add the Python-only ones (AS-4, AS-16,
AS-32).
"""

import threading
import time

import pycvc
from pycvc import process_status


def _raises(exc, fn, *args):
    try:
        fn(*args)
    except exc as e:
        return e
    raise AssertionError(f"{fn.__name__}{args!r} did not raise {exc.__name__}")


def _busy(seconds):
    """Use about `seconds` of this thread's CPU time (thread_time is per thread). Pure Python, so
    it lets the GIL go every switch interval."""
    end = time.thread_time() + seconds
    n = 0
    while time.thread_time() < end:
        n += 1
    return n


def _per_item():
    """Seconds per item of sum(range(n)): the best of a few runs, so one slow run (a cold
    interpreter, a frequency step) doesn't stretch every hold."""
    n = 1_000_000
    best = None
    for _ in range(5):
        start = time.perf_counter()
        sum(range(n))
        took = time.perf_counter() - start
        best = took if best is None else min(best, took)
    return max(best / n, 1e-10)


_PER_ITEM = _per_item()


def _hold_gil(seconds):
    """Run flat out for about `seconds` in one C call, which keeps the GIL all along."""
    sum(range(int(seconds / _PER_ITEM)))
    return 0


def _spin_cpu(seconds):
    """Run flat out, in short C calls that keep the GIL, until this thread has used `seconds` of
    CPU time, and return the CPU time it used: a CPU-bound kernel whose charges are checked
    against what it measured. (On Windows thread_time() advances at the scheduler tick, so it
    misses a thread that runs in bursts, like _busy's; one that runs flat out it counts well.)"""
    start = time.thread_time()
    while time.thread_time() - start < seconds:
        _hold_gil(0.005)
    return time.thread_time() - start


def _charged_steps(cpu_seconds, rate):
    """The steps a call's CPU time is charged at `rate` steps per CPU millisecond (at least one)."""
    return max(1, int(cpu_seconds * 1000.0 * rate))


class World:
    """A scheduler, an async host and an environment with intrinsics, driven by hand.

    kind "async": an async_scheduler whose kernels run on the app's background pool;
    kind "sync": the synchronous scheduler, whose kernels run inline in the calling step.
    """

    def __init__(self, kind="async", **register):
        self.app = pycvc.make_app()
        self.root = pycvc.State.instance(self.app)
        self.kind = kind
        if kind == "async":
            self.sched = pycvc.async_scheduler()
            self.host = pycvc.async_host(self.sched, self.app)
        else:
            self.sched = pycvc.scheduler()
            self.host = pycvc.async_host(self.sched)
        self.sched.set_watch_root(self.root)
        self.ctx = pycvc.intrinsics_context()
        self.ctx.sched = self.sched
        self.ctx.root = self.root
        self.env = pycvc.builtins.make_default_environment()
        pycvc.register_intrinsics(self.env, self.ctx)
        self.register = register

    def fn(self, name, kernel, timeout=None):
        pycvc.builtins.register_async_fn(self.env, self.host, name, kernel, "background", timeout)

    def submit(self, script, **limits):
        o = pycvc.execute_options()
        o.env = self.env
        for k, v in limits.items():
            setattr(o, k, v)
        return self.sched.execute(script, o)

    def pump(self, done, seconds=15.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.kind == "async":
                self.sched.drain_ingress()
                self.sched.sync_run(100000, 0.02)
            else:
                self.sched.run(100000, 0.02)
            if done():
                return True
            time.sleep(0.001)
        return done()

    def info(self, pid):
        return self.sched.get_process_info(pid)

    def finished(self, pid):
        return self.info(pid).status in (
            process_status.terminated, process_status.killed, process_status.failed)

    def run(self, script, **limits):
        pid = self.submit(script, **limits)
        assert self.pump(lambda: self.finished(pid)), f"process {pid} didn't end"
        return pid

    def result(self, pid):
        return self.sched.get_result(pid)

    def close(self):
        self.host.shutdown()


def _slow(x, ms=50):
    time.sleep(ms / 1000.0)
    return 2 * x


# ── calling (AS-1, 2, 5, 12, 14) ─────────────────────────────────────────────


def test_other_processes_step_while_one_is_parked():
    # AS-1, AS-2
    w = World()
    w.fn("slow-op", _slow)
    a = w.submit("(slow-op 2)")  # the whole program parks
    b = w.submit("(slow-op 3)")
    c = w.submit("(begin (set i 0) (while (< i 2000) (set i (+ i 1))) i)")
    assert w.pump(lambda: all(w.finished(p) for p in (a, b, c)))
    assert (w.result(a), w.result(b), w.result(c)) == (4, 6, 2000)
    w.close()


def test_await_a_future():
    # AS-5, on both schedulers
    for kind in ("async", "sync"):
        w = World(kind)
        w.fn("slow-op", _slow)
        pid = w.run("(begin (set f (slow-op-async 5)) (list (await f) (await f)))")
        assert w.result(pid) == [10, 10], kind
        w.close()


def test_inline_on_a_bare_evaluator_and_the_sync_scheduler():
    # AS-12: no executor; the same value and exceptions.
    host = pycvc.async_host()
    env = pycvc.builtins.make_default_environment()
    pycvc.builtins.register_async_fn(env, host, "slow-op", _slow)
    pycvc.builtins.register_async_fn(env, host, "boom", lambda: 1 // 0)
    for ev in (pycvc.stackless_evaluator(env), pycvc.evaluator(env)):
        assert ev.evaluate_script("(+ 1 (slow-op 2 5))") == 5
        got = ev.evaluate_script(
            '(try (boom) (catch (PythonError e) (list (get-attr e "python_type") '
            '(get-attr e "origin"))))')
        assert got == ["ZeroDivisionError", "async"]
    w = World("sync")
    w.fn("slow-op", _slow)
    assert w.result(w.run("(slow-op 4)")) == 8


def test_the_kernel_gets_a_copy_of_a_list_argument():
    # AS-14
    w = World()
    w.fn("sum-later", lambda xs: (time.sleep(0.05), sum(xs))[1])
    pid = w.run("(begin (set l (list 1 2 3)) (set f (sum-later-async l)) (append l 4) (await f))")
    assert w.result(pid) == 6
    w.close()


# ── errors (AS-3, 4, 6, 7, 8, 16) ────────────────────────────────────────────


class KernelError(Exception):
    pass


def test_a_python_kernels_exception_raises_at_the_call():
    # AS-3, AS-4: a PythonError, origin "async", function and python_type set.
    w = World()

    def boom():
        raise KernelError("kernel failed")

    w.fn("boom", boom)
    pid = w.run('(try (boom) (catch (PythonError e) (list (get-attr e "origin") (get-attr e '
                '"function") (get-attr e "python_type"))))')
    assert w.result(pid) == ["async", "boom", f"{__name__}.KernelError"]
    w.close()


def test_exec_register_async_fn_raises_at_the_call():
    # AS-16: no {"__async_error__": ...} dict.
    ex = pycvc.Exec(pycvc.make_app())

    def boom(x):
        raise KernelError(f"bad {x}")

    ex.register_async_fn("boom", boom)
    assert ex.run('(try (boom 1) (catch (PythonError e) (get-attr e "message")))').endswith(
        "bad 1")
    e = _raises(KernelError, ex.run, "(boom 2)")  # the whole program; the original re-raised
    assert isinstance(e.__cause__, pycvc.dsl_error)
    assert e.__cause__.dsl_class == "PythonError"
    assert e.__cause__.origin == "async"
    # Its host is the one pycvc.builtins.register_async_fn takes.
    assert ex.async_host().in_flight() == 0


def test_await_times_out_and_the_future_still_finishes():
    # AS-6
    w = World()
    w.fn("slow-op", _slow)
    pid = w.run('(begin (set f (slow-op-async 2 200)) (set r (try (await f 0.01) (catch '
                '(TimeoutError e) "timeout"))) (list r (await f)))')
    assert w.result(pid) == ["timeout", 4]
    w.close()


def test_await_all_raises_the_first_failure_once_all_finish():
    # AS-7, on both schedulers
    for kind in ("async", "sync"):
        w = World(kind)
        done = {}

        def a(done=done):
            time.sleep(0.1)
            done["a"] = True
            return 1

        def b():
            raise ValueError("b failed")

        w.fn("a", a)
        w.fn("b", b)
        pycvc.builtins.register_fn(w.env, "a-done", lambda done=done: done.get("a", False))
        pid = w.run('(try (await-all (list (a-async) (b-async))) (catch (PythonError e) (list '
                    '(get-attr e "python_type") (a-done))))')
        assert w.result(pid) == ["ValueError", True], kind
        w.close()


def test_a_registration_timeout_raises_timeout_error():
    # AS-8: the kernel sees cancelled() only from C++; a Python kernel's call still times out.
    w = World()
    w.fn("slow-op", _slow, timeout=0.05)
    start = time.monotonic()
    pid = w.run('(try (slow-op 1 400) (catch (TimeoutError e) (get-attr e "function")))')
    assert w.result(pid) == "slow-op"
    assert time.monotonic() - start < 0.35
    w.close()


# ── cancellation, progress, snapshots (AS-9, 10, 13, 15) ─────────────────────


def test_killing_a_parked_process_drops_its_call():
    # AS-9, AS-10
    w = World()
    started = threading.Event()

    def slow():
        started.set()
        time.sleep(0.2)
        return 1

    w.fn("slow", slow)
    pid = w.submit("(slow)")
    assert w.pump(lambda: started.is_set())
    assert w.sched.async_calls_in_flight() == 1
    assert w.sched.kill(pid)
    assert w.sched.async_calls_in_flight() == 0
    w.host.wait_idle()  # releases the GIL, so the kernel can finish
    w.sched.drain_ingress()  # its late result is dropped
    assert w.info(pid).status == process_status.killed
    w.close()


def test_a_restored_call_cant_resume():
    # AS-15
    w = World()
    w.fn("slow-op", _slow)
    pid = w.submit('(try (slow-op 2 300) (catch (RuntimeError e) "lost"))')
    assert w.pump(lambda: w.info(pid).status == process_status.waiting)
    node = w.root.child("__as15__")
    assert w.sched.snapshot(pid, node)
    restored = w.sched.restore(node, w.env)
    assert w.pump(lambda: w.finished(restored) and w.finished(pid))
    assert w.result(restored) == "lost"
    assert w.result(pid) == 4
    w.close()


# ── budgets (AS-18..33) ──────────────────────────────────────────────────────


def test_a_cpu_bound_calls_charges():
    # AS-18 (and AS-26, inline on the sync scheduler): run time, CPU time, steps. The kernel
    # measures the CPU time it used; the call is charged that, and steps at the default rate.
    for kind in ("async", "sync"):
        w = World(kind)
        w.fn("busy", lambda: _spin_cpu(0.2))
        pid = w.run("(busy)")
        used, i = w.result(pid), w.info(pid)
        assert used >= 0.2
        assert i.async_calls == 1, kind
        assert 0.7 * used <= i.async_cpu_time <= 1.3 * used + 0.03, (kind, i.async_cpu_time, used)
        assert i.async_time >= i.async_cpu_time - 0.02, kind
        expected = _charged_steps(i.async_cpu_time, pycvc.kDefaultAsyncStepsPerCpuMs)
        assert abs(i.async_steps - expected) <= max(2, expected // 100), (kind, i.async_steps)
        assert i.step_count >= i.async_steps
        assert i.elapsed_time >= i.async_time - 1e-3
        w.close()


def test_running_out_of_time_in_a_call_kills():
    # AS-19, AS-25: a Python kernel can't check cancelled(); its run time stops the process.
    w = World()
    w.fn("long", lambda: (time.sleep(0.6), 1)[1])
    start = time.monotonic()
    pid = w.run('(try (long) (catch (e) "caught"))', max_time=0.1)
    assert time.monotonic() - start < 0.5
    i = w.info(pid)
    assert i.status == process_status.killed
    assert i.exit_error == "time_limit_exceeded"
    w.close()


def test_a_calls_steps_count_against_max_steps():
    # AS-20
    w = World()
    w.fn("busy", lambda: _busy(0.05) and 0)
    pid = w.run("(busy)", max_steps=1000)
    assert w.info(pid).exit_error == "max_steps_exceeded"
    w.root.child("state_exec.defaults.limits.async_steps_per_cpu_ms").value("0")
    pid = w.run("(busy)", max_steps=1000)
    assert w.info(pid).status == process_status.terminated
    assert w.info(pid).async_steps == 1
    w.close()


def test_a_sleeping_call_is_charged_its_run_time_but_no_cpu():
    # AS-21, AS-29 (on Windows, the thread CPU clock reads CPU time, not wall time)
    w = World()
    w.fn("slow-op", _slow)
    i = w.info(w.run("(slow-op 1 200)"))
    assert i.async_time >= 0.19
    assert i.async_cpu_time < 0.05
    assert 1 <= i.async_steps < 100000
    w.close()


def _nap_span(ms):
    """Sleep `ms` and return when the sleep began and ended (perf_counter: on Windows monotonic()
    advances only at the ~15.6 ms scheduler tick)."""
    began = time.perf_counter()
    time.sleep(ms / 1000.0)
    return [began, time.perf_counter()]


def test_futures_in_flight_are_each_charged():
    # AS-24: two calls in flight at once are each charged their run time, so the process is
    # charged more run time than passed while they ran. The kernels report when they ran, so a
    # slow pump doesn't blur it.
    w = World()
    w.fn("nap", _nap_span)
    pid = w.run("(begin (set a (nap-async 100)) (set b (nap-async 100)) "
                "(list (await a) (await b)))")
    (a0, a1), (b0, b1) = w.result(pid)
    i = w.info(pid)
    assert i.async_calls == 2
    ran = (a1 - a0) + (b1 - b0)
    assert max(a1, b1) - min(a0, b0) < 0.75 * ran  # they overlapped
    assert i.async_time >= 0.9 * ran, (i.async_time, ran)
    w.close()


def test_the_step_rate_is_live_and_set_by_precedence():
    # AS-27, AS-28: the steps are the call's CPU time at the rate that wins.
    w = World()
    w.sched.set_id("t")
    w.fn("busy", lambda: _spin_cpu(0.05))
    w.root.child("state_exec.defaults.limits.async_steps_per_cpu_ms").value("10")
    w.root.child("state_exec.schedulers.t.limits.async_steps_per_cpu_ms").value("100")
    i = w.info(w.run("(busy)"))
    expected = _charged_steps(i.async_cpu_time, 100)  # the scheduler's node wins
    assert i.async_cpu_time >= 0.035 and abs(i.async_steps - expected) <= 2, (i.async_steps, expected)
    assert w.root.child("state_exec.schedulers.t.effective.async_steps_per_cpu_ms").value() == "100"
    i = w.info(w.run("(busy)", async_steps_per_cpu_ms=1.0))
    expected = _charged_steps(i.async_cpu_time, 1)  # the process's own wins
    assert abs(i.async_steps - expected) <= 2, (i.async_steps, expected)
    w.close()


def test_a_python_kernels_wait_for_the_gil_isnt_charged():
    # AS-32: another thread holds the GIL for 300 ms, in one C call, while the kernel's thread waits
    # to take it for its call. The kernel's run time is its own, not the wait.
    # - `hold` waits for the GIL while this thread holds it, so it takes the GIL the moment
    #   sync_run lets it go, before the call even starts; the kernel's thread then waits for it.
    # - The kernel is time.monotonic: a C callable, which never lets the GIL go once it has it (a
    #   Python function could, at an eval-breaker check, and wait again inside its call, a wait
    #   the host doesn't see), and which says when it ran. Ran before the hold, it took the GIL
    #   first: there was no wait, and the scenario is tried again.
    # - perf_counter, not monotonic, which on Windows advances only at the ~15.6 ms tick.
    stalled = 0
    for _ in range(20):
        w = World()
        w.fn("stamp", time.perf_counter)
        go = threading.Event()
        window = {}

        def hold(go=go, window=window):
            go.wait()
            window["from"] = time.perf_counter()
            _hold_gil(0.3)
            window["to"] = time.perf_counter()

        t = threading.Thread(target=hold)
        t.start()
        pid = w.submit("(stamp)")
        go.set()  # `hold` now waits for the GIL
        w.sched.sync_run(10, 0.01)  # `hold` takes the GIL; the call starts; its kernel waits
        t.join()
        assert w.pump(lambda w=w, pid=pid: w.finished(pid))
        ran_at, i = w.result(pid), w.info(pid)
        w.close()
        if ran_at < window["from"]:
            continue  # the kernel took the GIL before `hold` did: no wait to leave out
        if window["to"] - window["from"] > 0.45:
            # The machine stalled the threads (the hold took far longer than it runs): a stall
            # is run time, rightly charged, which would hide what this checks.
            stalled += 1
            continue
        assert ran_at - window["from"] >= 0.2  # it waited out the hold
        assert i.async_time < 0.1, i.async_time  # charged, the wait would be about 0.3 s
        return
    if stalled:
        print("  skipped: the machine stalled every try that waited for the GIL")
        return
    raise AssertionError("the kernel never had to wait for the GIL")


def test_the_counters_survive_a_snapshot():
    # AS-33
    w = World()
    w.fn("slow-op", _slow)
    pid = w.submit('(begin (slow-op 1 100) (msg-recv "never"))')
    assert w.pump(lambda: w.info(pid).async_calls == 1 and w.sched.async_calls_in_flight() == 0
                  and w.info(pid).status == process_status.waiting)
    node = w.root.child("__as33__")
    assert w.sched.snapshot(pid, node)
    restored = w.sched.restore(node, w.env)
    before, after = w.info(pid), w.info(restored)
    for f in ("async_calls", "async_time", "async_cpu_time", "async_steps", "step_count"):
        assert getattr(after, f) == getattr(before, f), f
    w.close()


def test_the_new_enum_values():
    assert int(pycvc.step_kind["async"]) == 3
    assert int(pycvc.eval_phase["await_call"]) == int(pycvc.eval_phase["raise_value"]) + 1


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc state_exec async call tests: OK")
