"""state_exec DSL execution + Python-authored DSL functions.

pycvc.Exec(app) runs DSL programs over app's state root (singleton-free — the
Exec is a handle bound to that app). register_fn(name, callable) makes a Python
callable invokable from DSL source: args arrive converted to Python
(int/float/bool/str/None/list/dict), the return converts back, and a Python
exception is contained (surfaced as a run() error, not a crash). Because the
Exec shares the app's state tree, DSL <-> Python <-> state all interoperate.
"""

import threading
import time

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
    assert (o.max_depth, o.step_tokens) == (100000, 0)  # C++ defaults
    o.max_depth = 32
    o.step_tokens = 8
    assert (o.max_depth, o.step_tokens) == (32, 8)


def test_exec_starts_from_the_host_parser_defaults():
    # The same defaults as the app's exec scheduler:
    # async_scheduler::host_parser_defaults().
    host = pycvc.async_scheduler.host_parser_defaults()
    assert (host.max_depth, host.step_tokens) == (100000, 1024)
    ex = pycvc.Exec(pycvc.make_app())
    d = ex.parser_defaults()
    assert (d.max_depth, d.step_tokens) == (host.max_depth, host.step_tokens)


def test_parse_options_from_state_resolves_like_cpp():
    # The real C++ function: it takes the tree root, a State, as in C++.
    app = pycvc.make_app()
    root = pycvc.State.instance(app)
    # Nothing set: the C++ default.
    assert pycvc.parse_options_from_state(root).step_tokens == 0
    # What Exec reads with: its own defaults under the tree.
    ex = pycvc.Exec(app)
    effective = pycvc.parse_options_from_state(root, "", ex.parser_defaults())
    assert effective.step_tokens == 1024

    root.child("state_exec.defaults.parser.max_depth").value("40")
    root.child("state_exec.schedulers.s1.parser.max_depth").value("50")
    assert pycvc.parse_options_from_state(root).max_depth == 40
    assert pycvc.parse_options_from_state(root, "s1").max_depth == 50
    root.child("state_exec.defaults.parser.max_depth").value("nope")  # bad: skipped
    assert pycvc.parse_options_from_state(root).max_depth == 100000


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
    # The real C++ function: it takes the tree root, a State, as in C++.
    app = pycvc.make_app()
    root = pycvc.State.instance(app)
    assert pycvc.data_options_from_state(root).step_nodes == 0
    ex = pycvc.Exec(app)
    assert pycvc.data_options_from_state(root, "", ex.data_defaults()).step_nodes == 1024
    root.child("state_exec.defaults.data.step_nodes").value("16")
    root.child("state_exec.schedulers.s1.data.step_nodes").value("8")
    assert pycvc.data_options_from_state(root).step_nodes == 16
    assert pycvc.data_options_from_state(root, "s1").step_nodes == 8
    root.child("state_exec.defaults.data.step_nodes").value("lots")  # bad: skipped
    assert pycvc.data_options_from_state(root).step_nodes == 0
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


def test_stdlib_collections_take_dsl_and_python_functions():
    # map/filter/reduce call a lambda as the program's own steps, and a registered Python
    # function like any native one. The list comes first, then the function.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    assert ex.run("(collections.map (list 1 2 3) (lambda (x) (* x x)))") == "(1 4 9)"
    assert ex.run("(collections.filter (collections.range 10) (lambda (x) (= (% x 3) 0)))") == (
        "(0 3 6 9)"
    )
    assert ex.run("(collections.reduce (list 1 2 3 4) (lambda (acc x) (+ acc (* x x))) 0)") == "30"
    ex.register_fn("twice", lambda x: x * 2)
    assert ex.run("(collections.map (list 1 2) twice)") == "(2 4)"
    # The builtins that call a function, apply and send, take a DSL function too.
    assert ex.run("(apply (lambda (a b) (* a b)) (list 6 7))") == "42"
    assert ex.run("(apply twice (list 21))") == "42"
    assert (
        ex.run(
            '(begin (set o (dict "__class__" (dict "inc" (lambda (self x) (+ x 1)))))'
            ' (send o "inc" 41))'
        )
        == "42"
    )


def test_stdlib_materializers_run_as_data_steps():
    # Exec slices deep-data work 1024 nodes a step (the host default): a big join and split
    # spread over data steps, and the answer is the same however the work is sliced.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    prog = (
        '(begin (set parts (string.split (string.join (collections.map (collections.range 20000)'
        ' str) ",") ","))'
        " (list (length parts) (nth parts 12345) (get-attr (budget-usage) \"data_steps\")))"
    )

    def data_steps(step_nodes=None):
        if step_nodes is not None:
            pycvc.state_set(app, "state_exec.defaults.data.step_nodes", str(step_nodes))
        out = ex.run(prog)
        assert out.startswith('(20000 "12345" '), out
        return int(out.rsplit(" ", 1)[1].rstrip(")"))

    host = data_steps()
    assert host > 100  # map, join and split, a slice a step
    assert data_steps(16) > host
    assert data_steps(0) < 10  # whole, in the steps that need it


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


def test_exec_collect_cycles_collects_its_registry():
    # An Exec's programs track into its own registry (td 4.7), whichever thread runs them:
    # another registry's collection (a bare evaluator's) frees none of their garbage, and
    # Exec.collect_cycles() frees it (R1-mid S-m9). The first collection makes the run's own idle
    # collection wait out the due rule's 1 s throttle, so the garbage is still there after it.
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    ex.collect_cycles()
    ex.run(
        "(begin (set j 0) (while (< j 50) (begin (let ((g nil)) (set g (lambda () g))) "
        "(set j (+ j 1)))) j)"
    )
    before = ex.tracked_objects()
    pycvc.stackless_evaluator(pycvc.builtins.make_default_environment()).collect_cycles()
    assert ex.tracked_objects() == before, "another registry's collection reached the Exec's"
    stats = ex.collect_cycles()
    after = ex.tracked_objects()
    print(f"  tracked {before} -> {after}; freed {stats.freed} of {stats.candidates}")
    assert stats.freed >= 50 and before - after >= 50, (before, after, stats.freed)


def test_is02_spawned_child_calls_registered_function():
    # IS-2 (docs/STATE_EXEC_THREADED.md 8.5): a spawned child sees the Exec's registered functions
    # (and its intrinsics); the parent's top-level definitions persist into the next run().
    app = pycvc.make_app()
    ex = pycvc.Exec(app)
    seen = []
    ex.register_fn("note", lambda x: seen.append(x) or x)
    ex.run("(defun twice (x) (* 2 x))")
    out = ex.run('(begin (spawn "(begin (note 21) (state-set \\"is02\\" \\"child\\"))") '
                 "(sleep 0.05) (twice 21))")
    assert out == "42", out
    assert seen == [21], seen
    assert ex.run('(state-get "is02")') == "child"


# ── ExecPool (S1-Q5): what test_pycvc_exec_pool.py's AR-4 and IS-2 leave out ──


def _pool_error(fn, *args):
    try:
        fn(*args)
    except Exception as e:  # noqa: BLE001 - RuntimeError, pycvc.dsl_error or parse_error
        return str(e)
    return None


def test_exec_pool_order_async_and_first_failure():
    # At threads 0 every group is pumped on the caller. Program i runs on pooled Exec i % size,
    # chrooted at "<prefix>.<i>"; an async function registered on the pool parks and resumes its
    # program; a batch with failures runs every program and raises the first failure in program
    # order; the pool goes on working.
    app = pycvc.make_app()
    pool = pycvc.ExecPool(app, threads=0, size=3, prefix="poolord")
    try:
        # An explicit threads=0 is applied: CVC_EXEC_THREADS overrides only the tree's setting.
        assert pool.threads == 0, pool.threads
        assert pool.run_all(['(state-incr "n")'] * 6) == ["1", "1", "1", "2", "2", "2"]
        assert pycvc.Exec(app).run('(state-get "poolord.2.n")') == "2"
        pool.register_async_fn("twice", lambda x: 2 * x)
        assert pool.run_all([f"(+ (twice {i}) 1)" for i in range(5)]) == ["1", "3", "5", "7", "9"]
        seen = []
        pool.register_fn("note", lambda x: seen.append(x) or x)
        msg = _pool_error(pool.run_all,
                          ["(note 1)", '(error "first")', "(note 3)", "(car 1)", "(note 5)"])
        assert msg is not None and "first" in msg, msg
        assert sorted(seen) == [1, 3, 5], seen
        assert pool.run_all(["(+ 2 2)"]) == ["4"]
        assert _pool_error(pool.run_all, ["(+ 1"]) is not None  # a syntax error raises too
    finally:
        pool.close()


def test_exec_pool_frame_yields_keep_a_program_alive():
    # A program yielding a frame at a time ((await v)) for longer than idle_timeout makes progress
    # every round: Exec.run completes it, and so does run_all, at threads 0 and 2 (R1-mid S-M2).
    # It loops on the wall clock, so it outlasts idle_timeout however fast the rounds go.
    prog = '(begin (set t0 (now)) (while (< (- (now) t0) 0.3) (await 0)) "done")'
    ex = pycvc.Exec(pycvc.make_app(), idle_timeout=0.1)
    ex.register_fn("now", time.monotonic)
    assert ex.run(prog) == "done"
    for threads in (0, 2):
        pool = pycvc.ExecPool(pycvc.make_app(), threads=threads, size=1, idle_timeout=0.1)
        try:
            pool.register_fn("now", time.monotonic)
            assert pool.run_all([prog]) == ["done"], threads
        finally:
            pool.close()


def test_exec_pool_frame_waiters_wait_for_the_frame():
    # A program yielding a frame at a time waits for each next frame, at most about 1 ms, as
    # Exec.run makes it, instead of being pumped again at once (the R1 skeptic's N2): over a fixed
    # wall time its frame count stays far below a busy loop's (about 200000 a second here), at
    # threads 0 and 2. The ceiling allows one frame each 0.2 ms; the beat gives about 300.
    prog = ('(begin (set n 0) (set t0 (now)) (while (< (- (now) t0) 0.3) (begin (await 0) '
            "(set n (+ n 1)))) n)")
    ex = pycvc.Exec(pycvc.make_app())
    ex.register_fn("now", time.monotonic)
    print(f"  Exec.run: {ex.run(prog)} frames in 0.3 s")
    for threads in (0, 2):
        pool = pycvc.ExecPool(pycvc.make_app(), threads=threads, size=1)
        try:
            pool.register_fn("now", time.monotonic)
            frames = int(pool.run_all([prog])[0])
            print(f"  ExecPool(threads={threads}): {frames} frames in 0.3 s")
            assert 0 < frames <= 1500, (threads, frames)
        finally:
            pool.close()


def test_exec_pool_size_follows_the_pools_threads():
    # size defaults to the app pool's real thread count, at least 1, not to a request the pool
    # did not apply (R1-mid S-m11): here the pool was built at 0, so a later threads=4 gets one
    # pooled Exec, which runs both programs.
    app = pycvc.make_app()
    first = pycvc.ExecPool(app, threads=0, size=1, prefix="sizefirst")
    later = pycvc.ExecPool(app, threads=4, prefix="sizelater")
    try:
        assert later.threads == 0, later.threads
        assert later.run_all(['(state-incr "n")'] * 2) == ["1", "2"]
    finally:
        later.close()
        first.close()


def test_exec_pool_idle_limit():
    # A pooled program parked with nothing pending is killed at the pool's idle_timeout, as in
    # Exec.run; the pooled Exec goes on working.
    app = pycvc.make_app()
    pool = pycvc.ExecPool(app, threads=2, size=2, idle_timeout=0.2)
    try:
        t0 = time.monotonic()
        msg = _pool_error(pool.run_all, ['(begin (msg-recv "nobody"))', "(+ 1 1)"])
        took = time.monotonic() - t0
        print(f"  killed after {took:.3f} s")
        assert msg is not None and "parked with no pending work" in msg, msg
        assert took < 10.0, took
        assert pool.run_all(["(+ 1 2)", "(+ 2 2)"]) == ["3", "4"]
    finally:
        pool.close()


def test_exec_pool_close_and_calls_during_a_batch():
    # While a batch runs on another thread, run_all and register_fn raise; close() kills what the
    # batch runs, waits for it, and the batch raises; afterwards the calls raise "closed". From a
    # function the batch calls, close() and run_all raise instead of waiting for themselves.
    app = pycvc.make_app()
    pool = pycvc.ExecPool(app, threads=2, size=2)
    started = threading.Event()
    pool.register_fn("started", lambda: started.set() or True)
    box = {}

    def batch():
        box["error"] = _pool_error(pool.run_all, ['(begin (started) (msg-recv "never"))', "1"])

    t = threading.Thread(target=batch, name="pool-batch")
    t.start()
    try:
        assert started.wait(10.0), "the batch never started"
        busy = "in progress"
        assert busy in (_pool_error(pool.run_all, ["1"]) or ""), "a second run_all ran"
        assert busy in (_pool_error(pool.register_fn, "x", lambda: 1) or ""), "registered"
    finally:
        pool.close()
        t.join(10.0)
    assert not t.is_alive(), "close() did not end the batch"
    assert "the pool was closed" in (box.get("error") or ""), box
    assert "the pool is closed" in (_pool_error(pool.run_all, ["1"]) or "")
    pool.close()  # idempotent

    inner = pycvc.ExecPool(app, size=1, prefix="poolinner")
    try:
        inner.register_fn("close-me", lambda: inner.close() or True)
        inner.register_fn("again", lambda: inner.run_all(["1"]) or True)
        msg = _pool_error(inner.run_all, ["(close-me)"])
        assert msg is not None and "from inside its run_all" in msg, msg
        msg = _pool_error(inner.run_all, ["(again)"])
        assert msg is not None and "in progress" in msg, msg
        assert inner.run_all(["(+ 1 1)"]) == ["2"]
    finally:
        inner.close()


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc exec + python-DSL-fn tests: OK")
