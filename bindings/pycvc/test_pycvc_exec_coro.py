"""pycvc.Exec.register_async_coro: an `async def` coroutine registered as a DSL
function is scheduled on an asyncio loop the Exec owns and stepped one bounded
slice per pump iteration, so the coroutine and the state_exec program march along
together — the coroutine's `await asyncio.sleep`/IO progresses between DSL slices,
and the parked DSL process resumes with the coroutine's return value.

Contrast register_async_fn (a BLOCKING callable offloaded to a pool worker thread):
register_async_coro runs the coroutine ON the run() thread, cooperatively.
"""

import asyncio

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


def test_coro_exception_becomes_error_dict():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)

    async def boom(_):
        raise ValueError("coro-kaboom")

    ex.register_async_coro("boom", boom)
    out = ex.run('(begin (get-attr (boom 1) "__async_error__"))')
    assert "coro-kaboom" in out, out

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


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc exec coroutine tests: OK")
