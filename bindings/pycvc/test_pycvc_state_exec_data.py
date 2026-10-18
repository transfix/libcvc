"""state_exec parity, part 4: deep-data jobs, the heap, and the stdlib registry.

The deep-data jobs (data_job.h, #658/#665), the heap share and reference-cycle collector
(heap.h, #667) and stdlib_registry, under the C++ names. The value conversion itself has no
depth limit and keeps sharing (test_pycvc_state_exec.py). docs/PYCVC_STATE_EXEC_PARITY.md has
the inventory.
"""

import pycvc
from pycvc import symbol


def _raises(exc, fn, *args):
    try:
        fn(*args)
    except exc as e:
        return e
    raise AssertionError(f"{fn.__name__}{args!r} did not raise {exc.__name__}")


def _builtins():
    return pycvc.builtins.make_default_environment()


# ── deep-data jobs ──────────────────────────────────────────────────────


def test_a_job_runs_a_slice_at_a_time():
    job = pycvc.make_copy_job([1, [2, [3, [4]]]])
    assert isinstance(job, pycvc.data_job)
    slices = 1
    while not job.step(1):
        slices += 1
    assert slices > 1 and job.nodes() > 0
    assert job.result() == [1, [2, [3, [4]]]]


def test_the_job_factories():
    run = pycvc.run_to_completion
    assert run(pycvc.make_equal_job([1, [2]], [1, [2]])) is True
    assert run(pycvc.make_equal_job([1], [2])) is False
    assert run(pycvc.make_print_job([symbol("a"), 1])) == pycvc.to_string([symbol("a"), 1])
    assert run(pycvc.make_concat_job(["a", 1, "b"], "-", "py-join")) == "a-1-b"
    tmpl = [symbol("+"), symbol("x"), [symbol("x")]]
    assert run(pycvc.make_substitute_job(tmpl, {"x": 41})) == [symbol("+"), 41, [41]]
    assert pycvc.is_compound([1]) and not pycvc.is_compound([]) and not pycvc.is_compound(5)
    assert pycvc.kDataNodeBytes == 256
    pycvc.count_data_nodes(3)  # outside an evaluator step: nothing to count against


def test_run_data_job_outside_an_evaluator_runs_it_whole():
    assert pycvc.run_data_job(pycvc.make_copy_job([1, 2])) == [1, 2]
    assert pycvc.run_data_job(pycvc.make_copy_job([1, 2]), len) == 2  # `then` maps the result
    job = pycvc.make_copy_job([1])
    pycvc.run_data_job(job)  # the job is given up to C++
    assert not job.thisown


def test_a_python_function_calls_dsl_functions():
    # call_function from a Python function a program calls: a native function is called
    # there; a lambda runs as the process's own frames, and the call returns its result.
    def apply_one(fn, arg):
        return pycvc.call_function(fn, [arg], "py-apply")

    env = _builtins()
    pycvc.builtins.register_fn(env, "apply-one", apply_one)
    for sched in (pycvc.scheduler(), pycvc.async_scheduler()):
        run = sched.run if isinstance(sched, pycvc.scheduler) else sched.sync_run
        o = pycvc.execute_options()
        o.env = env
        native = sched.execute("(apply-one string.length \"four\")", o)
        closure = sched.execute("(apply-one (lambda (x) (* x 2)) 21)", o)
        results = run()
        assert results[native] == 4 and results[closure] == 42
    # Outside a running program a lambda can't be called.
    lam = pycvc.stackless_evaluator(_builtins()).evaluate_script("(lambda (x) x)")
    e = _raises(RuntimeError, pycvc.call_function, lam, [1], "py-outside")
    assert "py-outside" in str(e)


def test_call_function_with_a_continuation():
    # The #683 overload: `then` gets the call's value once the function has run, however the
    # evaluator ran it (a lambda runs as frames, so `then` comes after them).
    def apply_then(fn, arg):
        return pycvc.call_function(fn, [arg], "py-then", lambda v: v + 1)

    env = _builtins()
    pycvc.builtins.register_fn(env, "apply-then", apply_then)
    s = pycvc.scheduler()
    o = pycvc.execute_options()
    o.env = env
    pid = s.execute("(apply-then (lambda (x) (* x 2)) 20)", o)
    assert s.run()[pid] == 41


def test_generator_jobs_and_nesting_limits():
    ev = pycvc.stackless_evaluator(_builtins())
    gen = ev.evaluate_script(
        "(generator (lambda () (begin (yield 1) (yield 2) (yield 3))))")
    assert isinstance(gen, pycvc.value) and gen.type_name() == "generator"
    assert pycvc.run_to_completion(pycvc.make_generator_job(gen)) == [1]  # the next value
    assert pycvc.run_to_completion(pycvc.make_generator_job(gen, True)) == [2, 3]  # the rest
    assert pycvc.run_to_completion(pycvc.make_generator_job(gen)) == []  # exhausted
    _raises(RuntimeError, pycvc.make_generator_job, 5)
    assert pycvc.kMaxNativeNesting == 256
    assert pycvc.kNativeStackReserve in (128 << 10, 32 << 10)  # Windows, elsewhere


# ── the heap ────────────────────────────────────────────────────────────


def test_heap_settings_and_fields():
    app = pycvc.make_app()
    root = pycvc.State.instance(app)
    assert pycvc.heap_options().check_steps == 4096
    root.child("state_exec.defaults.limits.heap_check_steps").value("100")
    root.child("state_exec.schedulers.s1.limits.heap_check_steps").value("50")
    assert pycvc.heap_options_from_state(root).check_steps == 100
    assert pycvc.heap_options_from_state(root, "s1").check_steps == 50
    assert pycvc.heap_remeasure_bytes(0) == 64 << 10
    assert pycvc.heap_remeasure_bytes(1 << 20) == 1 << 19  # half the last measurement
    p = pycvc.make_process()
    assert (p.heap_bytes, p.heap_measurements, p.heap_check_steps) == (0, 0, 4096)
    assert not p.heap_measuring and not p.heap_measure_due(0)  # no memory budget
    assert isinstance(pycvc.measure_heap_share(p), int)
    assert pycvc.run_to_completion(pycvc.make_heap_share_job(p)) == pycvc.measure_heap_share(p)
    p.release_state()
    assert p.state.heap_measured is None
    pycvc.eval_alloc_note(10)  # outside a step: nothing armed to note it


def test_a_budgeted_process_is_measured():
    for sched in (pycvc.scheduler(), pycvc.async_scheduler()):
        run = sched.run if isinstance(sched, pycvc.scheduler) else sched.sync_run
        o = pycvc.execute_options()
        o.max_memory = 64 << 20
        pid = sched.execute("(begin (set big (collections.range 50000)) (length big))", o)
        assert run()[pid] == 50000
        info = sched.get_process_info(pid)
        assert info.heap_measurements >= 1 and info.heap_memory > 0


def test_cycles_are_collected_on_this_thread():
    ev = pycvc.stackless_evaluator(_builtins())
    ev.evaluate_script(
        "(begin (set i 0) (while (< i 300) (begin (let ((h nil)) (set h (lambda () h)))"
        " (set i (+ i 1)))) 0)")
    assert pycvc.tracked_objects() > 0
    stats = pycvc.collect_cycles()
    assert isinstance(stats, pycvc.cycle_stats) and stats.freed > 0
    assert isinstance(pycvc.cycle_collection_due(), bool)
    pycvc.note_process_exit()


def test_evaluator_global_env():
    env = _builtins()
    ev = pycvc.stackless_evaluator(env)
    assert ev.global_env().lookup("+") is not None


# ── stdlib_registry ─────────────────────────────────────────────────────


def test_the_standard_registry():
    reg = pycvc.stdlib_registry.standard()  # a copy of the shared one
    assert {"string", "bytes", "math", "collections"} <= set(reg.list_modules())
    assert "string.length" in reg.list_functions("string")
    assert isinstance(reg.lookup_qualified("string.length"), pycvc.value)
    assert _raises(KeyError, reg.lookup_qualified, "string.nope").args == ("string.nope",)
    shared = pycvc.stdlib_registry.shared_scope()
    assert shared.frozen and isinstance(shared.lookup("string.length"), pycvc.value)
    # Adding to the copy leaves the shared registry alone.
    reg.register_function("py", "py.twice", lambda x: 2 * x)
    assert "py" in reg.list_modules()
    assert "py" not in pycvc.stdlib_registry.standard().list_modules()


def test_a_registry_of_ones_own():
    reg = pycvc.stdlib_registry()
    reg.register_function("py", "py.twice", lambda x: 2 * x)
    reg.register_function("py", "py.len", reg.lookup_qualified("string.length"))  # a native
    env = pycvc.environment.extend(_builtins())
    reg.import_module("py", env)
    ev = pycvc.stackless_evaluator(_builtins())
    assert ev.evaluate_script('(list (py.twice 21) (py.len "abc"))', env) == [42, 3]
    only = pycvc.environment()
    reg.import_module("math", only, ["math.sqrt"])
    assert list(only.bindings) == ["math.sqrt"]
    _raises(RuntimeError, reg.import_module, "nope", only)
    _raises(RuntimeError, reg.import_module, "py", None)


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc state_exec data/heap/stdlib parity tests: OK")
