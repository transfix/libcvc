"""pycvc.ExecPool and the per-Exec groups of threaded execution, Stage 1
(docs/STATE_EXEC_THREADED.md 4.12; S1-Q4, S1-Q5).

ExecPool(app, threads=None, size=None, prefix="exec_pool", fresh_env=False,
idle_timeout=60.0) runs many programs at once on the app's exec_pool
(run_all, register_fn, register_async_fn, threads, close). The cases are S1.md
section 4's pycvc ones: GR-5, IS-2, AR-4, FF-3 and PO-1. The sanitizer jobs
run this script (ci.yml's SAN_PY_TESTS) and require each case's "ok:" line.
"""

import os
import sys
import threading
import time

import pycvc

# A case's outcome must come within this many seconds (a hang fails it); the times are printed.
WAIT = 10.0
# The sanitizer jobs' sizes: their builds run 5-15x slower.
SANITIZER = os.environ.get("CVC_LOCKS_SANITIZER") == "1"
# What a pump-thread-only call raises while another thread pumps (S1-Q4: kPumpedElsewhereError).
PUMPING = "scheduler: another thread is pumping it"
# What awaiting another scheduler's threaded future raises (S1-Q2: kForeignFutureError).
FOREIGN = "await: the future belongs to another scheduler"


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


def _join(t, seconds=6 * WAIT):
    t.join(seconds)
    assert not t.is_alive(), f"{t.name} did not end within {seconds} s"


def _error(fn, *args):
    """What fn(*args) raised, as text, or None when it returned."""
    try:
        fn(*args)
    except Exception as e:  # noqa: BLE001 - pycvc raises RuntimeError or pycvc.dsl_error
        return str(e)
    return None


def _fib(n):
    a, b = 0, 1
    for _ in range(n):
        a, b = b, a + b
    return a


# ── ExecPool (S1-Q5) ───────────────────────────────────────────────────


def test_ar04_run_all():
    # AR-4 (docs/STATE_EXEC_THREADED.md 8.6): ExecPool(app, threads=8).run_all of 64 fib programs
    # gives the right results, in program order. With a Python native, the results are right and
    # the native never runs twice at once: it takes the GIL for each call. (The switch interval is
    # raised for that part, so the interpreter does not hand the GIL over inside a native's few
    # bytecodes.) fresh_env=True gives each program a fresh env; by default top-level definitions
    # persist. Scaling is the bench's to record, not asserted here.
    app = pycvc.make_app()
    pool = pycvc.ExecPool(app, threads=8)
    n = 16 if SANITIZER else 64
    try:
        print(f"  ExecPool.threads {pool.threads}")
        progs = [
            "(begin (defun fib (n) (if (< n 2) n (+ (fib (- n 1)) (fib (- n 2))))) "
            f"(fib {10 + i % 8}))"
            for i in range(n)
        ]
        t0 = time.monotonic()
        out = pool.run_all(progs)
        print(f"  {n} fib programs: {time.monotonic() - t0:.3f} s")
        assert out == [str(_fib(10 + i % 8)) for i in range(n)], out

        state = {"active": 0, "peak": 0, "calls": 0}

        def pyadd(a, b):
            state["active"] += 1
            state["peak"] = max(state["peak"], state["active"])
            state["calls"] += 1
            try:
                return a + b
            finally:
                state["active"] -= 1

        pool.register_fn("pyadd", pyadd)
        old = sys.getswitchinterval()
        sys.setswitchinterval(1.0)
        try:
            out = pool.run_all(
                [f"(begin (set s 0) (for i (range 20) (set s (pyadd s i))) (+ s {i}))"
                 for i in range(n)]
            )
        finally:
            sys.setswitchinterval(old)
        assert out == [str(190 + i) for i in range(n)], out
        assert state["calls"] == 20 * n, state
        assert state["peak"] == 1, state
    finally:
        pool.close()

    kept = pycvc.ExecPool(app, size=1, prefix="ar04kept")
    fresh = pycvc.ExecPool(app, size=1, prefix="ar04fresh", fresh_env=True)
    try:
        assert kept.run_all(["(begin (set x 5) x)"]) == ["5"]
        assert kept.run_all(["x"]) == ["5"]
        assert fresh.run_all(["(begin (set x 5) x)"]) == ["5"]
        msg = _error(fresh.run_all, ["x"])
        assert msg is not None and "undefined symbol" in msg, msg
    finally:
        kept.close()
        fresh.close()
    msg = _error(kept.run_all, ["1"])
    assert msg is not None and "the pool is closed" in msg, msg


def test_is02_in_a_pooled_exec():
    # IS-2 (docs/STATE_EXEC_THREADED.md 8.5) in a pooled Exec, test_pycvc_exec.py's case on an
    # ExecPool of one: a spawned child calls the pool's registered function, the parent's
    # top-level definitions persist into the next run_all, and the child's write lands under the
    # pooled Exec's chroot ("<prefix>.0").
    app = pycvc.make_app()
    pool = pycvc.ExecPool(app, threads=2, size=1, prefix="is02pool")
    seen = []
    try:
        pool.register_fn("note", lambda x: seen.append(x) or x)
        pool.run_all(["(defun twice (x) (* 2 x))"])
        out = pool.run_all(
            ['(begin (spawn "(begin (note 21) (state-set \\"is02\\" \\"child\\"))") '
             "(sleep 0.05) (twice 21))"]
        )
        assert out == ["42"], out
        assert seen == [21], seen
        assert pool.run_all(['(state-get "is02")']) == ["child"]
    finally:
        pool.close()
    assert pycvc.Exec(app).run('(state-get "is02pool.0.is02")') == "child"


# ── Per-Exec registries (td 4.7) ───────────────────────────────────────


def test_gr05_exec_from_short_lived_threads():
    # GR-5 (docs/STATE_EXEC_THREADED.md 8.2): an Exec run from 100 short-lived Python threads in
    # turn, each program leaving a live cycle in the Exec's env (keep<i>) and making 200 garbage
    # ones. The Exec's registry holds every live one, whichever thread made it, and stays bounded
    # (collected as it goes, never the 20000 garbage cycles); the ASan leg finds no leak.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    base = ex.tracked_objects()
    threads = 20 if SANITIZER else 100
    garbage = 200
    peak = base
    t0 = time.monotonic()
    for i in range(threads):
        prog = (
            f"(begin (set keep{i} (let ((h nil)) (set h (lambda () h)) h)) (set j 0) "
            f"(while (< j {garbage}) (begin (let ((g nil)) (set g (lambda () g))) "
            f"(set j (+ j 1)))) {i})"
        )
        t, out = _in_thread(f"gr05-{i}", ex.run, prog)
        _join(t)
        assert out.get("value") == str(i), out
        peak = max(peak, ex.tracked_objects())
    after = ex.tracked_objects()
    print(f"  {threads} threads: {time.monotonic() - t0:.2f} s; tracked {base} -> {after}, "
          f"peak {peak}")
    assert after - base >= threads, "a thread's live cycle is not in the Exec's registry"
    assert peak - base <= threads + 4 * 256 + garbage, "the Exec's registry was not collected"


# ── Futures across Execs (S1-Q2) ───────────────────────────────────────


def test_ff03_future_across_execs_raises():
    # FF-3 (S1.md section 4), pycvc's half: Exec A's threaded future, handed to Exec B through
    # Python, awaited in B: B raises "await: the future belongs to another scheduler" at once,
    # rather than waiting out its idle limit (20 s) for a completion that goes to A. A's own await
    # still gets the value.
    app = pycvc.make_app()
    a = pycvc.Exec(app)
    b = pycvc.Exec(app, idle_timeout=20.0)
    gate = threading.Event()
    stashed = threading.Event()
    box = {}

    def stash(f):
        box["f"] = f
        stashed.set()
        return True

    a.register_async_fn("gate", lambda: gate.wait(60.0) and 7)
    a.register_fn("stash", stash)
    b.register_fn("fetch", lambda: box["f"])
    t, out = _in_thread("ff03-a", a.run, "(begin (set f (gate-async)) (stash f) (await f))")
    try:
        assert stashed.wait(WAIT), "A never stashed its future"
        t0 = time.monotonic()
        msg = _error(b.run, "(begin (await (fetch)))")
        took = time.monotonic() - t0
        print(f"  B's await ended after {took:.3f} s")
        assert msg is not None and FOREIGN in msg, msg
        assert took < WAIT, took
    finally:
        gate.set()
        _join(t)
    assert out.get("value") == "7", out


# ── The pump owner (S1-Q4) ─────────────────────────────────────────────


def test_po01_raw_call_while_another_thread_pumps():
    # PO-1 (S1.md section 4): while Exec.run pumps on thread A (its program parked on msg-recv),
    # a raw ex.scheduler().kill(pid) from this thread raises RuntimeError "scheduler: another
    # thread is pumping it", and so does ex.register_fn; stop() and is_running() work throughout;
    # a monitor thread's ex.set_priority calls, made across run()'s quiesce, are answered (or told
    # "Exec.run ended"), never refused. Once run() has returned, the raw call and register_fn work.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    sched = ex.scheduler()
    box = {}
    started = threading.Event()

    def on_start():
        box["pid"] = ex.scheduler().current_pid()  # run()'s own thread: no owner but it
        started.set()
        return True

    ex.register_fn("started", on_start)
    halt = threading.Event()
    answers = []

    def monitor():
        pid = box["pid"]
        while not halt.is_set():
            try:
                answers.append(ex.set_priority(pid, 5))
            except RuntimeError as e:
                answers.append(str(e))

    t, out = _in_thread(
        "po01-run", ex.run, '(begin (started) (msg-recv "po01.go") (for i (range 5) (sleep 0)) 1)'
    )
    mon = None
    try:
        assert started.wait(WAIT), "the program never started"
        pid = box["pid"]
        mon = threading.Thread(target=monitor, name="po01-monitor")
        mon.start()
        sched.is_running()
        sched.stop()
        msg = _error(ex.register_fn, "late", lambda: 41)
        assert msg is not None and PUMPING in msg, msg
        msg = _error(sched.kill, pid)
        assert msg is not None and PUMPING in msg, msg
        sched.is_running()
        sched.stop()
    finally:
        sched.post_message("po01.go", "go")
        _join(t)
        time.sleep(0.05)  # the monitor runs on past run()'s return
        halt.set()
        if mon is not None:
            _join(mon)
    assert out.get("value") == "1", out
    assert answers, "the monitor made no call"
    for a in answers:
        assert a in (True, False) or a == "Exec.run ended", a
    assert sched.kill(pid) is False  # run() reaped it
    ex.register_fn("late", lambda: 41)
    assert ex.run("(+ (late) 1)") == "42"

    # run() still releases the GIL: this thread runs Python while another's run() steps a loop.
    ex2 = pycvc.Exec(app)
    begun = threading.Event()
    ending = threading.Event()
    ex2.register_fn("begun", lambda: begun.set() or True)
    ex2.register_fn("ending", lambda: ending.set() or True)
    n = 30000 if SANITIZER else 300000
    t, out = _in_thread(
        "po01-gil", ex2.run,
        f"(begin (begun) (set i 0) (while (< i {n}) (set i (+ i 1))) (ending) i)"
    )
    assert begun.wait(WAIT), "the long run never began"
    spins = 0
    t0 = time.monotonic()
    while not ending.is_set() and t.is_alive():
        spins += 1
    took = time.monotonic() - t0
    _join(t)
    print(f"  this thread ran {spins} iterations in {took:.3f} s of another thread's run()")
    assert out.get("value") == str(n), out
    assert spins > 100, spins


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc exec_pool tests: OK")
