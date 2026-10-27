"""pycvc.Exec.register_async_coro: an `async def` coroutine registered as a DSL
function is scheduled on an asyncio loop the Exec owns and stepped one bounded
slice per pump iteration, so the coroutine and the state_exec program march along
together — the coroutine's `await asyncio.sleep`/IO progresses between DSL slices,
and the parked DSL process resumes with the coroutine's return value.

Contrast register_async_fn (a BLOCKING callable offloaded to a pool worker thread):
register_async_coro runs the coroutine ON the run() thread, cooperatively.
"""

import asyncio
import time

import pycvc


def test_coro_awaits_then_resumes_with_result():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)

    async def slow_double(n):
        await asyncio.sleep(0.02)  # a real await, driven by the pump's asyncio slice
        return n * 2

    ex.register_async_coro("adbl", slow_double)
    assert ex.run("(begin (adbl 21))") == "42"


def test_coro_multiple_awaits_are_stepped_cooperatively():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)

    async def acount(n):
        total = 0
        for i in range(n):
            await asyncio.sleep(0.001)  # many awaits -> many cooperative steps
            total += i
        return total

    ex.register_async_coro("acount", acount)
    assert ex.run("(begin (acount 10))") == "45"  # sum(0..9)


def test_coro_marches_along_with_a_busy_state_exec_sibling():
    # The coroutine keeps awaiting while a co-runnable DSL sibling (a (while t 1) loop) runs. Each
    # pump iteration steps state_exec a slice AND the asyncio loop a slice, so the coroutine still
    # completes and the main program resumes — proving they interleave rather than starve each other.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)

    async def acount(n):
        total = 0
        for i in range(n):
            await asyncio.sleep(0.002)
            total += i
        return total

    ex.register_async_coro("acount", acount)
    out = ex.run('(begin (spawn "(while t 1)")'
                 '       (state-set "r" (str (acount 6)))'
                 '       (state-get "r"))')
    assert out == "15", out  # sum(0..5)


def test_coro_exception_raises_at_the_call():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)

    async def boom(_):
        raise ValueError("coro-kaboom")

    ex.register_async_coro("boom", boom)
    # The coroutine's exception raises at the call, a PythonError a try catches (no
    # {"__async_error__": ...} dict); uncaught, run() re-raises it.
    out = ex.run('(try (boom 1) (catch (PythonError e) (list (get-attr e "python_type") '
                 '(get-attr e "origin"))))')
    assert out == '("ValueError" "async")', out
    try:
        ex.run("(boom 1)")
        assert False, "expected the ValueError"
    except ValueError as e:
        assert "coro-kaboom" in str(e)

    # Exec stays usable after a contained coroutine error.
    async def idn(v):
        return v

    ex.register_async_coro("idn", idn)
    assert ex.run("(begin (idn 5))") == "5"


def test_coro_and_offload_and_sync_compose():
    # All three registration modes coexist in one Exec and one program.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    ex.register_fn("inc", lambda n: n + 1)  # sync, inline
    ex.register_async_fn("triple", lambda n: n * 3)  # offload to a pool worker

    async def plus_ten(n):
        await asyncio.sleep(0.005)
        return n + 10

    ex.register_async_coro("plus10", plus_ten)
    # inc(triple(4)) -> inc(12) -> 13 ; then plus10 awaits on it -> 23
    assert ex.run("(begin (plus10 (inc (triple 4))))") == "23"


def test_async_def_rejected_by_sync_and_pool_paths():
    # An async def belongs in register_async_coro; the sync/pool paths reject it up front so a
    # mistake surfaces as an error, not an un-awaited-coroutine repr string.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)

    async def acoro(n):
        return n

    for reg in (ex.register_fn, ex.register_async_fn):
        try:
            reg("bad", acoro)
            assert False, "expected async def to be rejected"
        except Exception as e:
            assert "async def" in str(e), str(e)


def test_reentrant_run_is_rejected():
    # A registered fn that calls back into run() must fail fast (one Exec, one thread), not corrupt
    # the scheduler. The re-entrant error is contained and surfaces as the program error.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)

    def reenter(_):
        ex.run("(+ 1 1)")  # illegal: re-entrant
        return 0

    ex.register_fn("reenter", reenter)
    try:
        ex.run("(begin (reenter 1))")
        assert False, "expected a re-entrant run to raise"
    except Exception as e:
        assert "re-entrant" in str(e), str(e)
    # The Exec is still usable after the contained re-entrancy error.
    assert ex.run("(+ 2 3)") == "5"


def _pump_slices(clock_resolution):
    """Run a program parked on a coroutine that sleeps 0.25 s, on an Exec whose asyncio loop takes
    `clock_resolution` as its clock's (asyncio runs a timer due within it at once). Returns when
    each of the pump's asyncio slices began and ended (time.perf_counter())."""
    real_new_event_loop = asyncio.new_event_loop
    slices = []

    def new_event_loop():
        loop = real_new_event_loop()
        loop._clock_resolution = clock_resolution
        run_forever = loop.run_forever

        def timed_run_forever():
            # A callback ready as the slice starts, as the Proactor loop (Windows' default) has at
            # each run_forever: its first iteration polls without waiting.
            loop.call_soon(lambda: None)
            began = time.perf_counter()
            try:
                return run_forever()
            finally:
                slices.append((began, time.perf_counter()))

        loop.run_forever = timed_run_forever
        return loop

    asyncio.new_event_loop = new_event_loop  # what the Exec makes its loop with
    try:
        ex = pycvc.Exec(pycvc.make_app())

        async def nap():
            await asyncio.sleep(0.25)
            return 7

        ex.register_async_coro("nap", nap)
        assert ex.run("(nap)") == "7"
    finally:
        asyncio.new_event_loop = real_new_event_loop
    return slices


def _median(xs):
    xs = sorted(xs)
    return xs[len(xs) // 2]


def test_a_waiting_asyncio_slice_is_the_pumps_wait():
    # Where asyncio's clock is fine (Linux, macOS, Windows on Python 3.13+), its 5 ms slice waits in
    # its selector until the stop timer: that is the pump's wait, and the next slice follows at
    # once. A 5 ms beat after each slice used to hold asyncio off half the time.
    slices = _pump_slices(1e-9)
    assert len(slices) >= 5, slices
    assert _median(end - began for began, end in slices) >= 0.004  # the slices waited
    gaps = [b[0] - a[1] for a, b in zip(slices, slices[1:])]
    assert _median(gaps) < 0.002, gaps


def test_an_asyncio_slice_that_returns_at_once_gets_a_beat():
    # Where asyncio's clock is coarse (Windows before Python 3.13: ~15.6 ms), it runs the slice's
    # stop timer at once, and the slice returns without waiting: the pump waits a 5 ms beat (no
    # CPU) between slices instead of spinning through them.
    slices = _pump_slices(0.05)
    assert _median(end - began for began, end in slices) < 0.0025  # the slices didn't wait
    gaps = [b[0] - a[1] for a, b in zip(slices, slices[1:])]
    assert _median(gaps) >= 0.002, gaps
    assert len(slices) < 200, len(slices)


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc exec coroutine tests: OK")
