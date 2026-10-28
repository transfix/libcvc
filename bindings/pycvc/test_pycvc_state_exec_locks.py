"""state_exec's lock phases from Python (docs/STATE_EXEC_LOCKS.md).

L0: the atomic natives state-incr, state-cas and state-swap, run through
pycvc.Exec over the app's state tree.

L2: pycvc.set_thread_class (PR-9, design 7.6).
"""

import sys
import threading

import pycvc

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
