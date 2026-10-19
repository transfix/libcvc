"""state_exec parity, part 3: evaluators, evaluator state, processes, policy, intrinsics.

pycvc.stackless_evaluator and pycvc.evaluator are the real C++ evaluators, with
evaluator_state, eval_frame, evaluation_stats, the process struct, the environment
and evaluator-state codec, resource_policy, and register_intrinsics over an
intrinsics_context. Most tests repeat a C++ test from src/cvc/tests
(state_exec_stackless_evaluator_test.cpp, state_exec_evaluator_test.cpp,
state_exec_codec_test.cpp, state_exec_intrinsics_test.cpp).
docs/PYCVC_STATE_EXEC_PARITY.md has the inventory.
"""

import gc
import threading
import time

import pycvc
from pycvc import eval_phase, process_status, symbol


def _raises(exc, fn, *args):
    try:
        fn(*args)
    except exc as e:
        return e
    raise AssertionError(f"{fn.__name__}{args!r} did not raise {exc.__name__}")


def _builtins():
    return pycvc.builtins.make_default_environment()


LOOP = (
    "(begin (set sum 0) (set i 0)"
    " (while (< i 100) (begin (set sum (+ sum i)) (set i (+ i 1)))) sum)"
)


# ── evaluators ──────────────────────────────────────────────────────────


def test_both_evaluators_evaluate():
    for ev in (pycvc.stackless_evaluator(_builtins()), pycvc.evaluator(_builtins())):
        assert ev.evaluate_script("(+ 1 2)") == 3
        assert ev.evaluate([symbol("*"), 6, 7]) == 42  # an expression, as evaluate(value_t)
        assert ev.evaluate("text") == "text"  # a str value evaluates to itself
        done = []
        assert ev.evaluate_script(LOOP, None, None, done.append) == 4950
        assert done == [4950]
        env = pycvc.environment.extend(_builtins())
        env.set("x", 40)
        assert ev.evaluate_script("(+ x 2)", env) == 42


def test_state_steps_to_a_result():
    ev = pycvc.stackless_evaluator(_builtins())
    st = ev.create_state("(begin (set x 1) (+ x 41))")  # a str is a script, as in C++
    assert isinstance(st, pycvc.evaluator_state) and not st.done
    steps = 0
    while not ev.step(st):
        steps += 1
    assert st.done and st.result == 42 and st.stack == []
    assert st.stats.get_step_count() > steps > 0
    # A value is an expression.
    assert ev.run(ev.create_state([symbol("+"), 1, 2])) == 3


def test_run_stops_at_max_steps_and_exposes_the_frames():
    ev = pycvc.stackless_evaluator(_builtins())
    st = ev.create_state(LOOP)
    assert ev.run(st, 10) is None and not st.done  # nil: not done
    frames = st.stack
    assert frames and all(isinstance(f, pycvc.eval_frame) for f in frames)
    assert all(isinstance(f.phase, eval_phase) for f in frames)
    assert frames[0].expr[0] == symbol("begin")
    assert ev.run(st) == 4950 and st.done
    _raises(OverflowError, ev.run, ev.create_state("1"), -1)


def test_state_reading_its_script_in_slices():
    ev = pycvc.stackless_evaluator(_builtins())
    popts = pycvc.parse_options()
    popts.step_tokens = 2
    st = ev.create_state("(set x 20) (+ x 22)", None, popts)
    assert isinstance(st.reading, pycvc.script_reader)
    assert st.reading.source() == "(set x 20) (+ x 22)"
    ev.step(st)
    assert ev.run(st) == 42 and st.reading is None


def test_stats_is_a_reference_that_keeps_its_owner():
    st = pycvc.stackless_evaluator(_builtins()).create_state("(+ 1 2)")
    stats = st.stats
    del st
    gc.collect()
    assert stats.get_step_count() == 0 and not stats.is_complete()
    rec = pycvc.evaluator(_builtins())
    rec.evaluate_script("(+ 1 2)")
    s = rec.stats()
    del rec
    gc.collect()
    assert s.get_step_count() > 0 and s.elapsed_seconds() >= 0.0
    own = pycvc.evaluation_stats()
    own.start()
    own.increment_step()
    own.mark_complete()
    assert own.get_step_count() == 1 and own.is_complete()


def test_stats_by_kind_of_step():
    # #658 counts steps and time by kind: reading the script, evaluating, deep-data work.
    kind = pycvc.step_kind
    assert [k.name for k in kind] == ["eval", "read", "data"]
    s = pycvc.evaluation_stats()
    s.increment_step(kind.read)
    s.increment_step(kind.read)
    s.increment_step()  # eval, as in C++
    assert (s.steps(kind.read), s.steps(kind.eval), s.get_step_count()) == (2, 1, 3)
    assert s.last_step_kind() is kind.eval
    s.add_step_time(0.25)  # charged to the last step's kind
    s.add_data_nodes(5)
    assert s.step_time(kind.eval) == 0.25 and s.step_time(kind.read) == 0.0
    assert s.data_nodes() == 5
    ev = pycvc.stackless_evaluator(_builtins())
    st = ev.create_state("(+ 1 2)")
    ev.run(st)
    assert st.stats.steps(kind.eval) > 0 and st.data_step_nodes == 0


def test_user_macros():
    ev = pycvc.evaluator(_builtins())
    assert ev.evaluate_script("(begin (defmacro double-it (x) (* x 2)) (double-it 21))") == 42
    assert ev.user_macros() == {"double-it": ([symbol("x")], [symbol("*"), symbol("x"), 2])}
    st = pycvc.evaluator_state()
    st.user_macros = {"m": ([symbol("a")], symbol("a"))}
    assert st.user_macros == {"m": ([symbol("a")], symbol("a"))}
    _raises(TypeError, setattr, st, "user_macros", {"m": (["a"], 1)})  # params are symbols


def test_interrupt_from_another_thread():
    # The evaluators release the GIL while they evaluate, so this thread can interrupt them.
    for ev in (pycvc.stackless_evaluator(_builtins()), pycvc.evaluator(_builtins())):
        t = threading.Timer(0.1, ev.interrupt)
        t.start()
        # The 10 s timeout only bounds a failure; the interrupt comes first.
        _raises(pycvc.evaluation_interrupted, ev.evaluate_script, "(while 1 1)", None, 10.0)
        t.join()
        ev.reset_interrupt()
        assert ev.evaluate_script("(+ 1 2)") == 3


def test_timeout_and_pause():
    for ev in (pycvc.stackless_evaluator(_builtins()), pycvc.evaluator(_builtins())):
        e = _raises(pycvc.evaluation_timeout, ev.evaluate_script, "(while 1 1)", None, 0.05)
        assert isinstance(e, RuntimeError)
        # A loop inside one step (a generator body that never yields) is cut the same way.
        e = _raises(
            pycvc.evaluation_timeout,
            ev.evaluate_script,
            "(next (generator (lambda () (while t 1))))",
            None,
            0.05,
        )
        assert "evaluation exceeded timeout" in str(e)
        ev.pause()
        assert ev.is_paused()
        t = threading.Timer(0.1, ev.resume)
        t.start()
        t0 = time.perf_counter()
        assert ev.evaluate_script("(+ 1 2)") == 3  # waits for resume()
        t.join()
        assert time.perf_counter() - t0 >= 0.05 and not ev.is_paused()


def test_restrict_special_forms():
    assert {"if", "begin", "quote", "defclass"} <= pycvc.stackless_evaluator.all_special_forms()
    ev = pycvc.stackless_evaluator(_builtins())
    ev.restrict_special_forms(["if", "begin"])
    assert ev.evaluate_script("(if #t 1 2)") == 1
    _raises(RuntimeError, ev.evaluate_script, "(quote 1)")
    assert ev.evaluate_script("(+ 1 2)") == 3  # not a special form
    ev.restrict_special_forms(None)
    assert ev.evaluate_script("(quote 1)") == 1
    _raises(TypeError, ev.restrict_special_forms, [1])


# ── codec ───────────────────────────────────────────────────────────────


def test_environment_and_evaluator_state_codec():
    app = pycvc.make_app()
    root = pycvc.State.instance(app)
    env = pycvc.environment.extend(pycvc.environment())
    env.set("a", [1, "two"])
    env.outer.set("b", symbol("c"))
    pycvc.encode_environment(root.child("snap.env"), env)
    back = pycvc.decode_environment(root.child("snap.env"))
    assert back.bindings == {"a": [1, "two"]} and back.lookup("b") == symbol("c")

    ev = pycvc.stackless_evaluator(_builtins())
    paused = ev.create_state(LOOP)
    ev.run(paused, 10)
    pycvc.encode_evaluator_state(root.child("snap.loop"), paused)
    restored = pycvc.decode_evaluator_state(root.child("snap.loop"))
    assert not restored.done and len(restored.stack) == len(paused.stack)

    # Resuming, as the C++ PauseResumeProducesCorrectResult does: the codec keeps each frame's
    # scope by value (not shared identity), so the expression mutates nothing; and native
    # builtins aren't serialized, so they go back on the root of every restored scope chain.
    st = ev.create_state("(+ 1 2 3 4 5 6 7 8 9 10)")
    ev.run(st, 3)
    assert not st.done
    pycvc.encode_evaluator_state(root.child("snap.es"), st)
    restored = pycvc.decode_evaluator_state(root.child("snap.es"))
    builtins = _builtins().bindings
    for scope in [restored.global_env] + [f.env for f in restored.stack]:
        while scope is not None and scope.outer is not None:
            scope = scope.outer
        if scope is not None:
            for name, fn in builtins.items():
                scope.set(name, fn)
    assert ev.run(restored) == 55


# ── process ─────────────────────────────────────────────────────────────


def test_process_struct():
    p = pycvc.make_process()
    assert isinstance(p, pycvc.process)
    assert (p.pid, p.status, p.priority, p.parent_pid) == (-1, process_status.ready, 0, -1)
    assert p.max_alloc_bytes == pycvc.kDefaultMaxAllocBytes == 16 << 20
    assert p.recv_path is None and p.exit_error is None and p.inbox == []
    assert p.time_left() is None  # no max_time
    assert p.alloc_limit(0) == 16 << 20
    p.max_memory = 1000
    assert p.alloc_limit(400) == 600  # the tighter of the two
    p.recv_path = "a.b"
    p.exit_error = "boom"
    p.signal_handlers = {"SIGUSR1": [symbol("begin"), 1]}
    p.pending_signals = ["SIGUSR1"]
    p.status = process_status.paused
    assert (p.recv_path, p.exit_error, p.status) == ("a.b", "boom", process_status.paused)
    assert p.signal_handlers == {"SIGUSR1": [symbol("begin"), 1]}
    assert list(p.pending_signals) == ["SIGUSR1"]
    ev = pycvc.watch_event()
    ev.watch_id = 3
    p.pending_watch_events = [ev]
    assert [e.watch_id for e in p.pending_watch_events] == [3]
    assert pycvc.process.watch_event is pycvc.watch_event
    entry = pycvc.watch_handler_entry()
    entry.handler = [symbol("begin"), 1]
    entry.path = "a.b"
    p.watch_handlers = {3: entry}
    got = p.watch_handlers[3]
    assert (got.handler, got.path) == ([symbol("begin"), 1], "a.b")
    seen = []
    p.on_complete = seen.append
    assert p.on_complete == seen.append
    state = p.state  # a reference into the process, which it keeps alive
    del p
    gc.collect()
    assert not state.done and state.stack == []


def test_current_process_during_a_run():
    for sched in (pycvc.scheduler(), pycvc.async_scheduler()):
        run = sched.run if isinstance(sched, pycvc.scheduler) else sched.sync_run
        assert sched.current_process() is None  # nothing is being stepped
        seen = []
        env = _builtins()
        pycvc.builtins.register_fn(env, "whoami",
                                   lambda: seen.append(sched.current_process()) or 0)
        o = pycvc.execute_options()
        o.env = env
        o.name = "me"
        pid = sched.execute("(whoami)", o)
        run()
        assert len(seen) == 1 and seen[0].pid == pid and seen[0].name == "me"
        # Shared, as process_ptr is: it shows the process as it is now, finished.
        assert seen[0].status is process_status.terminated and seen[0].exit_code == 0
        sched.queue_watch_event(pid, pycvc.watch_event())  # accepted for a finished pid
        assert isinstance(sched, pycvc.scheduler_base)


def test_max_alloc_bytes_from_the_tree():
    app = pycvc.make_app()
    root = pycvc.State.instance(app)
    assert pycvc.scheduler().max_alloc_bytes() == pycvc.kDefaultMaxAllocBytes
    assert pycvc.max_alloc_bytes_from_state(root, "") == pycvc.kDefaultMaxAllocBytes
    root.child("state_exec.defaults.limits.max_alloc_bytes").value("1024")
    root.child("state_exec.schedulers.s1.limits.max_alloc_bytes").value("2048")
    assert pycvc.max_alloc_bytes_from_state(root, "") == 1024
    assert pycvc.max_alloc_bytes_from_state(root, "s1") == 2048
    assert pycvc.max_alloc_bytes_from_state(root, "s2", 7) == 1024
    s = pycvc.scheduler()
    s.set_watch_root(root)
    s.set_id("s1")
    s.load_settings()
    assert s.max_alloc_bytes() == 2048


def test_allocation_limit_seen_by_a_python_function():
    assert pycvc.eval_alloc_limit() == pycvc.kDefaultMaxAllocBytes  # no guard armed
    e = _raises(RuntimeError, pycvc.eval_alloc_check, 1 << 40, "py-fn")
    assert "py-fn" in str(e)
    pycvc.eval_alloc_check(10, "py-fn")  # under the limit
    assert pycvc.alloc_bytes(3, 4) == 12
    assert pycvc.alloc_bytes(1 << 40, 1 << 40) == 2**64 - 1  # saturates at SIZE_MAX
    # During a step, a Python function sees the limit the scheduler armed: the tighter of
    # limits.max_alloc_bytes and what is left of the process's max_memory.
    seen = []
    env = _builtins()
    pycvc.builtins.register_fn(env, "limit", lambda: seen.append(pycvc.eval_alloc_limit()) or 0)
    for sched in (pycvc.scheduler(), pycvc.async_scheduler()):
        run = sched.run if isinstance(sched, pycvc.scheduler) else sched.sync_run
        o = pycvc.execute_options()
        o.env = env
        o.max_memory = 4096
        sched.execute("(limit)", o)
        run()
    assert len(seen) == 2 and all(lim is not None and lim <= 4096 for lim in seen)


# ── resource_policy ─────────────────────────────────────────────────────


def _limits(steps=0, time_=0.0, memory=0, messages=0):
    lim = pycvc.process_limits()
    lim.max_steps, lim.max_time, lim.max_memory, lim.max_messages = steps, time_, memory, messages
    return lim


def test_validate_limits():
    mode = pycvc.resource_policy.mode
    assert [m.name for m in mode] == ["strict", "clamp", "warn"]
    policy = pycvc.resource_policy()
    assert policy.enforce is mode.clamp
    r = pycvc.validate_limits(policy, _limits(1000, 60.0, 1024, 50))
    assert (r.max_steps, r.max_time, r.max_memory, r.max_messages) == (1000, 60.0, 1024, 50)
    policy.max_steps_default = 500
    policy.max_time_default = 30.0
    assert (pycvc.validate_limits(policy, _limits()).max_steps,
            pycvc.validate_limits(policy, _limits()).max_time) == (500, 30.0)
    policy.max_steps_max = 100
    assert pycvc.validate_limits(policy, _limits(200)).max_steps == 100  # clamp
    policy.enforce = mode.strict
    _raises(RuntimeError, pycvc.validate_limits, policy, _limits(200))
    _raises(TypeError, setattr, policy, "enforce", 0)  # an enum argument must be the enum


# ── intrinsics ──────────────────────────────────────────────────────────


def _intrinsic_env(app, sched):
    ctx = pycvc.intrinsics_context()
    ctx.sched = sched
    ctx.root = pycvc.State.instance(app)
    ctx.pid = 1
    env = _builtins()
    pycvc.register_intrinsics(env, ctx)
    return env, ctx


def test_register_intrinsics_over_a_context():
    app = pycvc.make_app()
    sched = pycvc.scheduler()
    env, ctx = _intrinsic_env(app, sched)
    assert ctx.sched is sched and ctx.channels is None
    for name in ("state-get", "state-set", "msg-send", "msg-recv", "self-pid", "spawn"):
        assert isinstance(env.lookup(name), pycvc.value), name
    o = pycvc.execute_options()
    o.env = env
    pid = sched.execute('(begin (state-set "py.x" "hello") (state-get "py.x"))', o)
    assert sched.run()[pid] == "hello"
    assert pycvc.state_get(app, "py.x") == "hello"


def test_registered_intrinsics_keep_their_context_alive():
    app = pycvc.make_app()
    env, _ = _intrinsic_env(app, pycvc.scheduler())  # the context and scheduler go out of scope
    gc.collect()
    other = pycvc.scheduler()
    o = pycvc.execute_options()
    o.env = env
    pid = other.execute('(begin (state-set "kept.x" "yes") (state-get "kept.x"))', o)
    assert other.run()[pid] == "yes"


def test_apply_chroot_and_channels():
    app = pycvc.make_app()
    sched = pycvc.scheduler()
    ctx = pycvc.intrinsics_context()
    ctx.sched = sched
    pycvc.apply_chroot(ctx, pycvc.State.instance(app), "sandbox.user1")
    assert ctx.root_path == "sandbox.user1"
    env = _builtins()
    pycvc.register_intrinsics(env, ctx)
    o = pycvc.execute_options()
    o.env = env
    sched.execute('(state-set "x" "inside")', o)
    sched.run()
    assert pycvc.state_get(app, "sandbox.user1.x") == "inside"

    cp = pycvc.channel_policy()
    cp.enforce = True
    cp.declared = ["inbox"]
    cp.global_ = ["g"]
    ctx.channels = cp
    assert ctx.channels is cp and list(ctx.channels.declared) == ["inbox"]
    assert list(cp.global_) == ["g"]

    assert pycvc.resolve_channel_key("doc", "nav.done") == "doc.channels.nav.done"
    assert pycvc.resolve_channel_key("doc", "#tick") == "#tick"
    assert pycvc.resolve_channel_key("doc", "/g") == "g"
    assert pycvc.resolve_channel_key("", "x") == "x"
    fut = pycvc.make_future("ch.reply")
    assert fut == {"__future__": "ch.reply"}
    assert pycvc.future_channel_of(fut) == "ch.reply"
    assert pycvc.future_channel_of(5) is None


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc state_exec evaluator parity tests: OK")
