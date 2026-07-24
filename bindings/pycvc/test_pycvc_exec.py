"""state_exec DSL execution + Python-authored DSL functions.

pycvc.Exec(app) runs DSL programs over app's state root (singleton-free — the
Exec is a handle bound to that app). register_fn(name, callable) makes a Python
callable invokable from DSL source: args arrive converted to Python
(int/float/bool/str/None/list/dict), the return converts back, and a Python
exception is contained (surfaced as a run() error, not a crash). Because the
Exec shares the app's state tree, DSL <-> Python <-> state all interoperate.
"""

import pycvc


def _exec(app):
    """Exec(app), or None if this build lacks state_exec."""
    try:
        return pycvc.Exec(app)
    except Exception as e:
        if "without state_exec" in str(e):
            return None
        raise


# ── core DSL evaluation ─────────────────────────────────────────────────


def test_arithmetic_and_forms():
    app = pycvc.make_app()
    ex = _exec(app)
    if ex is None:
        print("  skip: build has no state_exec")
        return
    assert ex.run("(+ 2 3)") == "5"
    assert ex.run("(* 6 7)") == "42"
    assert ex.run("(begin (set x 5) x)") == "5"
    assert ex.run('(str-concat "ab" "cd")') == "abcd"
    assert ex.run("(< 1 2)") == "#t"
    assert ex.run("(> 1 2)") == "#f"


def test_parse_error_raises():
    app = pycvc.make_app()
    ex = _exec(app)
    if ex is None:
        return
    try:
        ex.run("(begin (set x")  # unbalanced
    except Exception:
        pass
    else:
        raise AssertionError("a parse error must raise")


# ── DSL <-> state <-> Python round trips (one shared app) ───────────────


def test_dsl_reads_and_writes_shared_state():
    app = pycvc.make_app()
    ex = _exec(app)
    if ex is None:
        return
    # Python writes state; DSL reads it.
    pycvc.state_set(app, "shared.x", "7")
    assert ex.run('(state-get "shared.x")') == "7"
    # DSL writes state; Python reads it.
    ex.run('(state-set "dsl.out" "wrote-from-dsl")')
    assert pycvc.state_get(app, "dsl.out") == "wrote-from-dsl"


# ── Python functions callable from DSL (the headline) ───────────────────


def test_python_function_called_from_dsl():
    app = pycvc.make_app()
    ex = _exec(app)
    if ex is None:
        return

    ex.register_fn("py-double", lambda x: x * 2)
    ex.register_fn("py-add", lambda a, b: a + b)
    ex.register_fn("py-greet", lambda name: "hi " + name)

    assert ex.run("(py-double 21)") == "42"
    assert ex.run("(py-add 40 2)") == "42"
    assert ex.run('(py-greet "cvc")') == "hi cvc"
    # composes with builtins: (+ 1 (py-double 20)) = 41
    assert ex.run("(+ 1 (py-double 20))") == "41"


def test_python_dsl_function_touches_state():
    app = pycvc.make_app()
    ex = _exec(app)
    if ex is None:
        return

    # A Python DSL function that writes state on the shared app.
    def stash(key, value):
        pycvc.state_set(app, key, value)
        return "ok"

    ex.register_fn("stash", stash)
    assert ex.run('(stash "py.k" "py.v")') == "ok"
    assert pycvc.state_get(app, "py.k") == "py.v"  # visible outside the DSL

    # And one that reads state back.
    ex.register_fn("fetch", lambda k: pycvc.state_get(app, k))
    assert ex.run('(fetch "py.k")') == "py.v"


def test_python_exception_is_contained():
    app = pycvc.make_app()
    ex = _exec(app)
    if ex is None:
        return

    def boom(*_):
        raise ValueError("kaboom")

    ex.register_fn("boom", boom)
    try:
        ex.run("(boom)")
    except Exception as e:
        assert "boom" in str(e) or "kaboom" in str(e), str(e)
    else:
        raise AssertionError("a raising Python DSL fn must surface an error, not crash")

    # The Exec still works afterward — the interpreter did not die.
    assert ex.run("(+ 2 2)") == "4"


def test_value_marshaling_types():
    app = pycvc.make_app()
    ex = _exec(app)
    if ex is None:
        return
    seen = {}

    def capture(x):
        seen["type"] = type(x).__name__
        seen["val"] = x
        return x

    ex.register_fn("capture", capture)
    assert ex.run("(capture 5)") == "5"
    assert seen["type"] == "int" and seen["val"] == 5
    assert ex.run('(capture "text")') == "text"
    assert seen["type"] == "str"
    ex.run("(capture #t)")
    assert seen["type"] == "bool" and seen["val"] is True


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc exec + python-DSL-fn tests: OK")
