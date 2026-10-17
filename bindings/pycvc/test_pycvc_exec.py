"""state_exec DSL execution + Python-authored DSL functions.

pycvc.Exec(app) runs DSL programs over app's state root (singleton-free — the
Exec is a handle bound to that app). register_fn(name, callable) makes a Python
callable invokable from DSL source: args arrive converted to Python
(int/float/bool/str/None/list/dict), the return converts back, and a Python
exception is contained (surfaced as a run() error, not a crash). Because the
Exec shares the app's state tree, DSL <-> Python <-> state all interoperate.
"""

import pycvc


# ── core DSL evaluation ─────────────────────────────────────────────────


def test_arithmetic_and_forms():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    assert ex.run("(+ 2 3)") == "5"
    assert ex.run("(* 6 7)") == "42"
    assert ex.run("(begin (set x 5) x)") == "5"
    assert ex.run('(str-concat "ab" "cd")') == "abcd"
    assert ex.run("(< 1 2)") == "#t"
    assert ex.run("(> 1 2)") == "#f"


def test_parse_error_raises():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    try:
        ex.run("(begin (set x")  # unbalanced
    except Exception:
        pass
    else:
        raise AssertionError("a parse error must raise")


def _run_error(ex, src):
    """run(src) must raise; return the message."""
    try:
        ex.run(src)
    except Exception as e:  # noqa: BLE001 - pycvc surfaces C++ errors as RuntimeError
        return str(e)
    raise AssertionError(f"{src!r} must raise")


def test_parse_error_names_the_problem():
    # Exec uses the same state_exec defaults as the app's exec scheduler: the script
    # is read inside its process, so a syntax error comes back as the process's exit
    # error. run() raises with the parse error's own message, as it did when the
    # script was parsed at submit.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    assert "unclosed parenthesis" in _run_error(ex, "(begin (set x")


def test_parser_settings_come_from_the_app_tree():
    # Exec's scheduler watches the app's state tree, so state_exec.defaults.parser.*
    # configures it exactly as it configures the app's exec scheduler
    # (docs/STATE_EXEC_GRAMMAR.md).
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    deep = "(+ 1 (+ 2 (+ 3 (+ 4 5))))"  # four levels of nesting
    assert ex.run(deep) == "15"

    pycvc.state_set(app, "state_exec.defaults.parser.max_depth", "3")
    assert "nesting deeper than 3" in _run_error(ex, deep)
    assert ex.run("(+ 1 (+ 2 3))") == "6"  # three levels still parse

    # Parsing at submit (step_tokens 0) gives the same results and the same errors.
    pycvc.state_set(app, "state_exec.defaults.parser.step_tokens", "0")
    assert "nesting deeper than 3" in _run_error(ex, deep)
    assert "unclosed parenthesis" in _run_error(ex, "(begin (set x")
    assert ex.run("(+ 1 (+ 2 3))") == "6"

    pycvc.state_set(app, "state_exec.defaults.parser.max_depth", "256")
    assert ex.run(deep) == "15"


# ── state_exec parser settings: the C++ surface, under the C++ names ────


def test_parse_options_is_the_cpp_struct():
    o = pycvc.parse_options()
    assert (o.max_depth, o.step_tokens) == (256, 0)  # C++ defaults
    o.max_depth = 32
    o.step_tokens = 8
    assert (o.max_depth, o.step_tokens) == (32, 8)


def test_exec_starts_from_the_host_parser_defaults():
    # The same defaults as the app's exec scheduler:
    # async_scheduler::host_parser_defaults().
    host = pycvc.async_scheduler.host_parser_defaults()
    assert (host.max_depth, host.step_tokens) == (256, 1024)
    ex = pycvc.Exec(pycvc.make_app())
    d = ex.parser_defaults()
    assert (d.max_depth, d.step_tokens) == (host.max_depth, host.step_tokens)


def test_parse_options_from_state_resolves_like_cpp():
    app = pycvc.make_app()
    # Nothing set: the C++ default.
    assert pycvc.parse_options_from_state(app).step_tokens == 0
    # What Exec reads with: its own defaults under the tree.
    ex = pycvc.Exec(app)
    effective = pycvc.parse_options_from_state(app, "", ex.parser_defaults())
    assert effective.step_tokens == 1024

    pycvc.state_set(app, "state_exec.defaults.parser.max_depth", "40")
    pycvc.state_set(app, "state_exec.schedulers.s1.parser.max_depth", "50")
    assert pycvc.parse_options_from_state(app).max_depth == 40
    assert pycvc.parse_options_from_state(app, "s1").max_depth == 50
    pycvc.state_set(app, "state_exec.defaults.parser.max_depth", "nope")  # bad: skipped
    assert pycvc.parse_options_from_state(app).max_depth == 256


def test_set_parser_defaults_changes_how_exec_parses():
    ex = pycvc.Exec(pycvc.make_app())
    o = pycvc.parse_options()
    o.max_depth = 2
    ex.set_parser_defaults(o)
    assert ex.parser_defaults().max_depth == 2
    assert ex.parser_defaults().step_tokens == 0
    assert "nesting deeper than 2" in _run_error(ex, "(+ 1 (+ 2 (+ 3 4)))")
    assert ex.run("(+ 1 (+ 2 3))") == "6"


# ── state_exec deep-data settings (docs/STATE_EXEC_GRAMMAR.md, "Deep data on
# the executor"), under the C++ names ────────────────────────────────────


def test_data_options_is_the_cpp_struct():
    o = pycvc.data_options()
    assert o.step_nodes == 0  # C++ default: deep-data work runs whole
    o.step_nodes = 64
    assert o.step_nodes == 64


def test_exec_starts_from_the_host_data_defaults():
    # The same defaults as the app's exec scheduler:
    # async_scheduler::host_data_defaults().
    host = pycvc.async_scheduler.host_data_defaults()
    assert host.step_nodes == 1024
    ex = pycvc.Exec(pycvc.make_app())
    assert ex.data_defaults().step_nodes == host.step_nodes


def test_data_options_from_state_resolves_like_cpp():
    app = pycvc.make_app()
    assert pycvc.data_options_from_state(app).step_nodes == 0
    ex = pycvc.Exec(app)
    assert pycvc.data_options_from_state(app, "", ex.data_defaults()).step_nodes == 1024
    pycvc.state_set(app, "state_exec.defaults.data.step_nodes", "16")
    pycvc.state_set(app, "state_exec.schedulers.s1.data.step_nodes", "8")
    assert pycvc.data_options_from_state(app).step_nodes == 16
    assert pycvc.data_options_from_state(app, "s1").step_nodes == 8
    o = pycvc.data_options()
    o.step_nodes = 0
    ex.set_data_defaults(o)
    assert ex.data_defaults().step_nodes == 0


def test_deep_data_gives_the_same_results_however_it_is_sliced():
    # Compare and print values nested 3000 deep: whole in one step, 1024 nodes a step (the host
    # default) or 4 nodes a step.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    prog = (
        "(begin (set a 1) (set b 1) (set i 0)"
        " (while (< i 3000) (begin (set a (list a)) (set b (list b)) (set i (+ i 1))))"
        " (list (= a b) (length (str a))))"
    )
    sliced = ex.run(prog)
    pycvc.state_set(app, "state_exec.defaults.data.step_nodes", "0")
    whole = ex.run(prog)
    pycvc.state_set(app, "state_exec.defaults.data.step_nodes", "4")
    assert ex.run(prog) == whole == sliced
    assert whole.startswith("(#t ")


# ── DSL <-> state <-> Python round trips (one shared app) ───────────────


def test_dsl_reads_and_writes_shared_state():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    # Python writes state; DSL reads it.
    pycvc.state_set(app, "shared.x", "7")
    assert ex.run('(state-get "shared.x")') == "7"
    # DSL writes state; Python reads it.
    ex.run('(state-set "dsl.out" "wrote-from-dsl")')
    assert pycvc.state_get(app, "dsl.out") == "wrote-from-dsl"


# ── Python functions callable from DSL (the headline) ───────────────────


def test_python_function_called_from_dsl():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)

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
    ex = pycvc.Exec(app)

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
    ex = pycvc.Exec(app)

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
    ex = pycvc.Exec(app)
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


def test_bytes_marshaling_round_trip():
    # A binary payload crosses Python -> DSL -> Python as native `bytes`, with no
    # lossy UTF-8 coercion. Exercises both directions of the value_t marshaling:
    # py_to_value (make-bytes return) and value_to_py (capture arg). The payload
    # carries a NUL and a 0xFF byte so a str() round-trip (the pre-fix behavior)
    # would corrupt it.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    blob = b"\x00\x01\xffhello\x00"
    seen = {}

    ex.register_fn("make-bytes", lambda: blob)

    def capture(x):
        seen["type"] = type(x).__name__
        seen["val"] = x
        return x

    ex.register_fn("capture", capture)

    ex.run("(capture (make-bytes))")
    assert seen["type"] == "bytes", seen["type"]
    assert seen["val"] == blob, seen["val"]


# ── the stdlib modules (string, bytes, math, collections) ───────────────


def test_stdlib_modules_are_always_loaded():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    assert ex.run('(string.length "café")') == "4"  # codepoints, not bytes
    assert ex.run('(string.upper "go")') == "GO"
    assert ex.run('(bytes.length (string.encode "café"))') == "5"
    assert ex.run("(math.floor 3.7)") == "3"
    assert ex.run("(collections.reduce (list 1 2 3) + 0)") == "6"
    # A program's own binding shadows a stdlib name without touching other programs.
    assert ex.run("(begin (set string.length 7) string.length)") == "7"
    assert pycvc.Exec(app).run('(string.length "ab")') == "2"


def test_stdlib_allocation_limit_comes_from_the_app_tree():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)

    def run_error(src):
        try:
            ex.run(src)
        except Exception as e:  # noqa: BLE001 - any surfaced DSL error
            return str(e)
        return ""

    # The 16 MiB default stops a huge one-call allocation.
    assert "allocation limit" in run_error("(collections.range 1000000000)")
    # limits.max_alloc_bytes on the app tree is read on every run.
    pycvc.state_set(app, "state_exec.defaults.limits.max_alloc_bytes", "1024")
    assert "collections.range" in run_error("(collections.range 1000)")
    pycvc.state_set(app, "state_exec.defaults.limits.max_alloc_bytes", "1048576")
    assert ex.run("(length (collections.range 1000))") == "1000"


# ── reference cycles ────────────────────────────────────────────────────


def test_exec_functions_survive_cycle_collection():
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    ex.run("(defun twice (x) (* 2 x))")
    # Each iteration leaks a let scope (a closure bound in the scope it captured). That many make
    # the scheduler collect cycles between steps, and Exec's own environment, held by Exec, must
    # come through intact.
    ex.run(
        "(begin (set i 0) (while (< i 1000) (begin (let ((h nil)) (set h (lambda () h))) "
        "(set i (+ i 1)))) i)"
    )
    assert ex.run("(twice 21)") == "42"


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc exec + python-DSL-fn tests: OK")
