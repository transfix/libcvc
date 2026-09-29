"""pycvc.Exec async DSL functions: register_async_fn offloads a Python callable
to the app's compute pool and the DSL program parks until it delivers, so the
scheduler stays live and the GIL is released while waiting.

These exercise the real async path end to end: a Python callable runs on a pool
WORKER thread (not the run() thread), the program resumes with its result, and a
separate Python thread makes progress while the async call is in flight (which is
only possible if run() releases the GIL — otherwise the worker could never even
acquire it to start, and this would deadlock).
"""

import threading
import time

import pycvc


def test_async_fn_offloads_and_resumes_with_result():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    seen = {}

    def double(n):
        seen["thread"] = threading.get_ident()
        return n * 2

    ex.register_async_fn("adouble", double)
    # Nested (not the whole program) so the self-park has an enclosing frame.
    out = ex.run("(begin (adouble 21))")
    assert out == "42", out
    # It ran on a pool worker, NOT the thread that called run().
    assert seen["thread"] != threading.get_ident(), "async fn should run off the run() thread"


def test_gil_released_so_other_python_thread_runs_during_async_wait():
    # The strong test: while the async callable is in flight, a SEPARATE Python
    # thread runs. This can only happen if run() releases the GIL during its idle
    # wait — and in fact, without that release the pool worker could never acquire
    # the GIL to start, so this program would deadlock rather than merely block the
    # bumper. Passing proves both the GIL discipline and the off-thread execution.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    started = threading.Event()
    release = threading.Event()
    counter = {"n": 0}

    def slow(x):
        started.set()  # we are now running on the pool worker
        release.wait(3.0)  # block (releasing the GIL) until the bumper frees us
        return x

    ex.register_async_fn("slow", slow)

    def bumper():
        started.wait(3.0)  # wait until slow() is actually running
        for _ in range(100):
            counter["n"] += 1
        release.set()  # let slow() finish

    t = threading.Thread(target=bumper)
    t.start()
    out = ex.run("(begin (slow 7))")
    t.join()
    assert out == "7", out
    assert counter["n"] == 100, counter["n"]


def test_async_fn_exception_becomes_error_dict():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)

    def boom(_):
        raise ValueError("kaboom-async")

    ex.register_async_fn("boom", boom)
    # A raise inside the async callable resumes the program with {"__async_error__": <msg>} — a
    # reserved key a normal dict result cannot collide with.
    out = ex.run('(begin (get-attr (boom 1) "__async_error__"))')
    assert "kaboom-async" in out, out
    # The Exec stays usable after a contained async error.
    ex.register_async_fn("idn", lambda v: v)
    assert ex.run("(begin (idn 5))") == "5"


def test_sync_and_async_fns_compose():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    ex.register_fn("inc", lambda n: n + 1)  # synchronous, inline
    ex.register_async_fn("triple", lambda n: n * 3)  # async, on the pool
    # (inc (triple 4)) -> inc(12) -> 13; the async result threads into the sync call.
    out = ex.run("(begin (inc (triple 4)))")
    assert out == "13", out


def test_async_resolves_with_a_corunnable_sibling():
    # Regression for the review's HIGH deadlock: the program spawns a sibling that stays RUNNABLE
    # (a busy (while t 1) loop) while it awaits an async fn. Before the fix, the pump kept the GIL
    # whenever ANY process was runnable, so the pool worker could never acquire the GIL to run the
    # Python body and deliver -> unbounded hang. Now the GIL is released around each slice, so the
    # worker runs and the program completes. (The sibling loops forever but the main pid terminates,
    # so run() returns; a short sleep in the worker guarantees the co-runnable window.)
    app = pycvc.make_app()
    ex = pycvc.Exec(app)

    def slow(x):
        time.sleep(0.02)  # ensure the sibling is co-runnable while this worker is in flight
        return x

    ex.register_async_fn("slow", slow)
    out = ex.run('(begin (spawn "(while t 1)")'
                 '       (state-set "r" (str (slow 7)))'
                 '       (state-get "r"))')
    assert out == "7", out


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc exec async DSL-fn tests: OK")
