"""state_exec's lock phases from Python (docs/STATE_EXEC_LOCKS.md).

L0: the atomic natives state-incr, state-cas and state-swap, run through
pycvc.Exec over the app's state tree.
"""

import pycvc


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


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc state_exec locks tests: OK")
