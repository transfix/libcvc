"""state_exec parity, part 2: environments, builtins and the schedulers.

pycvc.scheduler and pycvc.async_scheduler are the real C++ classes, with
execute_options, process_info, scheduler_stats, and the process_status and
scheduling_policy enums (enum.IntEnum classes). Most tests here repeat a C++ test
from src/cvc/tests/state_exec_scheduler_test.cpp or state_exec_async_test.cpp,
on both schedulers where both have the call. docs/PYCVC_STATE_EXEC_PARITY.md has
the inventory.
"""

import threading
import time

import pycvc
from pycvc import process_status, scheduling_policy, symbol


def _raises(exc, fn, *args):
    try:
        fn(*args)
    except exc as e:
        return e
    raise AssertionError(f"{fn.__name__}{args!r} did not raise {exc.__name__}")


def _schedulers():
    """A fresh sync and async scheduler, each with its run and step calls."""
    s = pycvc.scheduler()
    a = pycvc.async_scheduler()
    return [(s, s.run, s.step), (a, a.sync_run, a.sync_step)]


def _opts(**fields):
    o = pycvc.execute_options()
    for k, v in fields.items():
        setattr(o, k, v)
    return o


# ── enums ───────────────────────────────────────────────────────────────


def test_enums_are_int_enums_under_the_cpp_names():
    assert [m.name for m in process_status] == [
        "ready", "running", "paused", "waiting", "terminated", "killed"]
    assert [m.name for m in scheduling_policy] == ["round_robin", "priority", "priority_rr"]
    assert process_status.killed == 5  # an IntEnum, with the C++ values
    assert pycvc.to_string(process_status.waiting) == "waiting"
    assert pycvc.parse_process_status("paused") is process_status.paused
    assert pycvc.parse_process_status("bogus") is process_status.ready  # as in C++
    # A plain int is a value, not an enum, so to_string keeps both C++ overloads.
    assert pycvc.to_string(2) == "2"
    # An enum argument must be the enum.
    _raises(TypeError, pycvc.scheduler, 1)
    assert pycvc.scheduler(scheduling_policy.priority).policy() is scheduling_policy.priority
    assert pycvc.async_scheduler().policy() is scheduling_policy.round_robin


# ── environment and builtins ────────────────────────────────────────────


def test_environment_is_the_scope_chain():
    env = pycvc.environment()
    env.set("x", 1)
    assert env.lookup("x") == 1
    child = pycvc.environment.extend(env)
    assert child.outer is not None and child.outer.lookup("x") == 1
    assert child.lookup("x") == 1  # lookup walks the chain
    child.set("y", [1, "two"])
    assert child.bindings == {"y": [1, "two"]}  # this scope's own map
    child.set_existing("x", 5)  # the nearest scope that has it
    assert env.lookup("x") == 5 and "x" not in child.bindings
    child.set_existing("z", None)  # bound nowhere: the outermost unfrozen scope
    assert "z" not in child.bindings and env.bindings == {"x": 5, "z": None}
    # C++ returns nullptr for an unbound name; Python raises KeyError (None is nil).
    assert _raises(KeyError, child.lookup, "nope").args == ("nope",)
    env.bindings = {"a": symbol("b")}  # replaces the map, as assigning it does in C++
    assert env.bindings == {"a": symbol("b")}
    _raises(KeyError, child.lookup, "x")
    _raises(TypeError, setattr, env, "bindings", {1: 2})
    assert pycvc.environment.extend(None).outer is None


def test_a_frozen_scope_is_read_only():
    shared = pycvc.environment()
    shared.set("k", 1)
    assert not shared.frozen
    shared.frozen = True
    _raises(RuntimeError, shared.set, "k", 2)  # C++ throws std::logic_error
    child = pycvc.environment.extend(shared)
    assert child.lookup("k") == 1
    child.set_existing("k", 3)  # never written into the frozen scope: shadowed here instead
    assert child.bindings == {"k": 3} and shared.bindings == {"k": 1}


def test_builtins_and_register_fn():
    env = pycvc.builtins.make_default_environment()
    plus = env.lookup("+")
    assert isinstance(plus, pycvc.value) and plus.type_name() == "native_fn"
    calls = []

    def greet(name):
        calls.append(name)
        return "hi " + name

    pycvc.builtins.register_fn(env, "greet", greet)
    pycvc.builtins.register_fn(env, "plus", plus)  # a native fn moves as a pycvc.value
    for sched, run, _ in _schedulers():
        pid = sched.execute('(list (greet "ann") (plus 2 3))', _opts(env=env))
        assert run()[pid] == ["hi ann", 5]
    assert calls == ["ann", "ann"]
    # Not callable, no environment, or an async def: refused at registration.
    _raises(RuntimeError, pycvc.builtins.register_fn, env, "bad", 42)
    _raises(RuntimeError, pycvc.builtins.register_fn, None, "f", greet)

    async def coro():
        return 1

    _raises(RuntimeError, pycvc.builtins.register_fn, env, "c", coro)


def test_a_python_fn_error_surfaces_as_each_scheduler_reports_it():
    env = pycvc.builtins.make_default_environment()
    pycvc.builtins.register_fn(env, "boom", lambda: 1 / 0)
    # The sync scheduler throws from run(), as a throwing native fn does in C++.
    s = pycvc.scheduler()
    s.execute("(boom)", _opts(env=env))
    assert "division by zero" in str(_raises(RuntimeError, s.run))
    # The async scheduler kills the process and keeps the reason as its exit error.
    a = pycvc.async_scheduler()
    pid = a.execute("(boom)", _opts(env=env))
    assert a.sync_run() == {}
    assert a.get_process_info(pid).status is process_status.killed
    assert "division by zero" in a.get_exit_error(pid)
    assert a.reap(pid) and a.get_process_info(pid) is None
    assert a.get_exit_error(999) is None


def test_percent_encode_and_decode():
    assert pycvc.percent_encode("a b/é~") == "a%20b%2F%C3%A9~"
    assert pycvc.percent_decode("a%20b%2F%C3%A9~") == "a b/é~"
    assert pycvc.percent_decode("x+y%zz") == "x y%zz"  # '+' is a space; a bad escape stays


# ── execute_options, process_info, scheduler_stats ──────────────────────


def test_execute_options_has_the_cpp_fields_and_defaults():
    o = pycvc.execute_options()
    assert (o.name, o.priority, o.uid, o.gid, o.root_path, o.owner) == ("", 0, "", "", "", "")
    assert (o.max_steps, o.max_time, o.max_step_time, o.max_memory) == (0, 0.0, 0.0, 0)
    assert (o.max_messages, o.max_message_bytes) == (0, 0)
    assert o.signal_handlers == {} and o.env is None and o.on_complete is None
    o.max_memory = 2**63  # the uint64_t limits take the full range
    assert o.max_memory == 2**63
    _raises(OverflowError, setattr, o, "max_steps", -1)


def test_process_info_and_stats_report_each_process():
    for sched, run, _ in _schedulers():
        p1 = sched.execute("42", _opts(name="my-process", uid="user1", gid="group1"))
        p2 = sched.execute("(+ 1 2)")
        info = sched.get_process_info(p1)
        assert (info.pid, info.name, info.uid, info.gid) == (p1, "my-process", "user1", "group1")
        assert info.status is process_status.ready and info.parent_pid == -1
        assert sched.get_process_info(p2).name == "proc-%d" % p2
        assert sched.get_process_info(999) is None
        stats = sched.get_stats()
        assert (stats.total_processes, stats.ready, stats.terminated) == (2, 2, 0)
        run()
        stats = sched.get_stats()
        assert stats.terminated == 2 and stats.total_steps > 0
        procs = sched.list_processes()
        assert [p.pid for p in procs] == [p1, p2]  # sorted by pid
        assert all(p.status is process_status.terminated for p in procs)
        assert procs[1].step_count > 0


# ── running programs ────────────────────────────────────────────────────


def test_both_schedulers_run_programs():
    for sched, run, step in _schedulers():
        assert not sched.has_runnable() and sched.process_count() == 0
        p1 = sched.execute("(+ 1 2)")
        p2 = sched.execute("(* 3 4)")
        fib = sched.execute(
            "(begin (defun fib (n) (if (<= n 1) n (+ (fib (- n 1)) (fib (- n 2))))) (fib 10))")
        nil = sched.execute("nil")
        assert sched.has_runnable() and sched.process_count() == 4
        # No result yet: C++ returns std::nullopt, Python raises KeyError(pid).
        assert _raises(KeyError, sched.get_result, p1).args == (p1,)
        assert run() == {p1: 3, p2: 12, fib: 55, nil: None}
        assert sched.get_result(p1) == 3
        assert sched.get_result(nil) is None  # nil is a result, not a missing one
        assert sched.get_results() == {p1: 3, p2: 12, fib: 55, nil: None}
        assert not sched.has_runnable() and not sched.is_running()


def test_execute_takes_a_script_or_a_value():
    for sched, run, _ in _schedulers():
        script = sched.execute("(* 6 7)")
        expr = sched.execute(pycvc.parse("(* 6 7)"))  # an expression, as execute(value_t)
        quoted = sched.execute([symbol("quote"), [1, symbol("a")]])
        assert run() == {script: 42, expr: 42, quoted: [1, symbol("a")]}


def test_step_by_step_and_run_limits():
    for sched, run, step in _schedulers():
        pid = sched.execute("(+ 1 2)")
        stepped = 0
        while sched.has_runnable():
            step()
            stepped += 1
        assert stepped > 0 and sched.get_result(pid) == 3
        sched.execute("(begin 1 2 3 4 5 6 7 8 9 10)")
        run(3)  # at most three steps in all
        assert sched.has_runnable()
        _raises(OverflowError, run, -1)
        run(None, 30.0)
        assert not sched.has_runnable()


def test_priority_policies():
    for policy in (scheduling_policy.priority, scheduling_policy.priority_rr):
        for sched in (pycvc.scheduler(policy), pycvc.async_scheduler(policy)):
            run = sched.run if isinstance(sched, pycvc.scheduler) else sched.sync_run
            low = sched.execute("(begin 1 2 3)", _opts(priority=10))
            high = sched.execute("(begin 4 5 6)", _opts(priority=-10))
            assert run() == {low: 3, high: 6}


def test_process_control():
    for sched, run, step in _schedulers():
        pid = sched.execute("(begin 1 2 3 4 5)")
        step()
        assert sched.pause(pid)
        assert sched.get_process_info(pid).status is process_status.paused
        assert step() == 0  # nothing runnable while paused
        assert sched.resume(pid) and not sched.resume(pid)
        assert sched.get_process_info(pid).status is process_status.ready
        assert sched.set_priority(pid, -5) and sched.get_process_info(pid).priority == -5
        assert sched.sleep(pid, 0.01)
        assert sched.get_process_info(pid).status is process_status.waiting
        # A sleeper counts as runnable for the sync scheduler only; the async one's driver
        # steps it after the deadline (as the C++ tests do).
        assert sched.has_runnable() is isinstance(sched, pycvc.scheduler)
        time.sleep(0.03)
        for _ in range(64):
            if sched.get_process_info(pid).status is process_status.terminated:
                break
            step()
        assert sched.get_result(pid) == 5
        assert not sched.kill(pid)  # already terminated
        for call in (sched.pause, sched.kill, sched.yield_frame):
            assert not call(999)
        assert sched.fork(999) == -1

        doomed = sched.execute("(begin 1 2 3 4 5)")
        step()
        assert sched.kill(doomed)
        assert sched.get_process_info(doomed).status is process_status.killed


def test_fork_copies_the_process():
    for sched, run, step in _schedulers():
        parent = sched.execute("(begin 1 2 3)", _opts(priority=-5, uid="user1", gid="group1"))
        step()
        child = sched.fork(parent)
        assert child > 0 and child != parent
        info = sched.get_process_info(child)
        assert (info.parent_pid, info.priority) == (parent, -5)
        assert (info.uid, info.gid) == ("user1", "group1")
        assert info.status is process_status.ready
        run()
        assert sched.get_result(parent) == 3 and sched.get_result(child) == 3


def test_limits_kill_the_process():
    for sched, run, step in _schedulers():
        capped = sched.execute("(begin 1 2 3 4 5 6 7 8 9 10)", _opts(max_steps=5))
        later = sched.execute("(begin 1 2 3 4 5 6 7 8 9 10)")
        assert sched.set_max_steps(later, 3)
        assert sched.set_max_time(later, 10.0) and sched.set_max_memory(later, 1 << 20)
        assert sched.set_max_messages(later, 100) and sched.set_max_message_bytes(later, 4096)
        assert not sched.set_max_steps(999, 1)
        info = sched.get_process_info(later)
        assert (info.max_time, info.max_memory, info.max_messages, info.max_message_bytes) == (
            10.0, 1 << 20, 100, 4096)
        run()
        assert sched.get_process_info(capped).status is process_status.killed
        assert sched.get_process_info(later).status is process_status.killed


def test_signals():
    handler = [symbol("begin"), 42]
    for sched, run, step in _schedulers():
        o = _opts(signal_handlers={"SIGUSR1": handler})
        assert o.signal_handlers == {"SIGUSR1": handler}
        pid = sched.execute("(begin 1 2 3 4 5)", o)
        step()
        assert sched.send_signal(pid, "SIGUSR1")
        run()
        assert sched.get_process_info(pid).status is process_status.terminated
        assert not sched.send_signal(pid, "SIGUSR1")  # terminated
        doomed = sched.execute("(begin 1 2 3 4 5 6 7 8 9 10)")
        step()
        assert sched.send_signal(doomed, "SIGKILL")
        assert sched.get_process_info(doomed).status is process_status.killed
        assert not sched.send_signal(999, "SIGUSR1")


def test_on_complete_is_a_python_callable():
    results = []

    def done(value):
        results.append(value)
        return {1, 2}  # ignored, so not converted

    o = _opts(on_complete=done)
    assert o.on_complete is done
    for sched, run, _ in _schedulers():
        sched.execute("(list 1 2)", o)
        run()
    assert results == [[1, 2], [1, 2]]
    o.on_complete = None
    assert o.on_complete is None
    _raises(RuntimeError, setattr, o, "on_complete", 42)
    # A raising on_complete is raised from the call that finished the process.
    s = pycvc.scheduler()
    s.execute("7", _opts(on_complete=lambda v: 1 / 0))
    assert "division by zero" in str(_raises(RuntimeError, s.run))


def test_messages_queue_until_received():
    for sched, run, step in _schedulers():
        # No receiver: the message queues under its path (FIFO).
        assert sched.deliver_to_receivers("test.q", {"n": 1}) == 0
        sched.deliver_to_receivers("test.q", {"n": 2})
        assert sched.pending_message_count("test.q") == 2 and sched.total_pending_messages() == 2
        assert sched.pop_pending_message("test.q") == {"n": 1}
        assert sched.pop_pending_message("test.q") == {"n": 2}
        # Empty: C++ returns std::nullopt, Python raises KeyError(path).
        assert _raises(KeyError, sched.pop_pending_message, "test.q").args == ("test.q",)
        # A waiting receiver gets the message instead, and wakes.
        pid = sched.execute("(begin 1 2 3 4 5)")
        step()
        assert sched.receive_message(pid, "test.inbox")
        assert sched.get_process_info(pid).status is process_status.waiting
        assert sched.deliver_to_receivers("test.inbox", "hello") == 1
        assert sched.get_process_info(pid).status is process_status.ready
        assert not sched.receive_message(999, "test.inbox")
        # The queue is bounded by max_pending_messages.
        assert sched.max_pending_messages == 1024
        sched.max_pending_messages = 2
        for n in range(4):
            sched.deliver_to_receivers("test.b", n)
        assert sched.pending_message_count("test.b") == 2


def test_watch_handlers_register_and_unregister():
    app = pycvc.make_app()
    for sched, run, step in _schedulers():
        sched.set_watch_root(pycvc.State.instance(app))
        pid = sched.execute("(begin 1 2 3)")
        sched.register_watch_handler(pid, 1, [symbol("begin"), 1], "watched.path")
        sched.unregister_watch_handler(pid, 1)
        run()
        assert sched.get_result(pid) == 3
        sched.set_watch_root(None)


def test_scheduler_settings_come_from_the_tree():
    app = pycvc.make_app()
    root = pycvc.State.instance(app)
    root.child("state_exec.defaults.max_pending_messages").value("512")
    root.child("state_exec.schedulers.py-sched.max_pending_messages").value("2048")
    root.child("state_exec.schedulers.py-sched.parser.max_depth").value("12")
    s = pycvc.scheduler()
    assert s.parser_settings().max_depth == 256
    s.set_watch_root(root)
    s.set_id("py-sched")
    assert s.id() == "py-sched"
    s.load_settings()
    assert s.max_pending_messages == 2048
    assert s.parser_settings().max_depth == 12
    assert pycvc.scheduler.settings_root == "state_exec"
    assert pycvc.scheduler.defaults_prefix == "state_exec.defaults"
    assert pycvc.scheduler.sched_prefix == "state_exec.schedulers"


def test_data_settings_and_the_budget_by_kind():
    # #658's deep-data settings sit beside the parser settings, and process_info splits a
    # process's steps and time by kind.
    a = pycvc.async_scheduler()
    assert a.data_defaults().step_nodes == 0  # a bare async_scheduler does deep data in place
    assert pycvc.async_scheduler.host_data_defaults().step_nodes == 1024
    d = pycvc.data_options()
    d.step_nodes = 7
    a.set_data_defaults(d)
    assert a.data_defaults().step_nodes == 7
    assert a.data_settings().step_nodes == 7  # no watch root: the defaults
    assert pycvc.scheduler().data_settings().step_nodes == 0
    for sched, run, _ in _schedulers():
        pid = sched.execute("(+ 1 2)")
        run()
        info = sched.get_process_info(pid)
        assert info.eval_steps > 0 and info.read_steps == 0 and info.data_steps == 0
        assert info.eval_time >= 0.0 and info.data_nodes == 0


# ── async_scheduler's own calls ─────────────────────────────────────────


def test_async_scheduler_extras():
    a = pycvc.async_scheduler()
    assert a.parser_defaults().step_tokens == 0  # a bare async_scheduler parses at submit
    assert pycvc.async_scheduler.host_parser_defaults().step_tokens == 1024
    a.set_parser_defaults(pycvc.async_scheduler.host_parser_defaults())
    assert a.parser_defaults().step_tokens == 1024

    a1 = a.execute("(begin 1 2 3 4 5)", _opts(owner="docA"))
    a2 = a.execute("(begin 1 2 3 4 5)", _opts(owner="docA"))
    b1 = a.execute("(begin 1 2 3 4 5)", _opts(owner="docB"))
    a.sync_step()
    assert a.kill_owner("docA") == 2 and a.kill_owner("docA") == 0
    assert a.get_process_info(a1).status is process_status.killed
    assert a.get_process_info(a2).status is process_status.killed
    assert a.get_process_info(b1).status is not process_status.killed

    # (await ...) parking: only wake_awaiting, once per frame, readies it.
    assert a.yield_frame(b1)
    a.sync_step()
    assert a.get_process_info(b1).status is process_status.waiting
    assert a.wake_awaiting() == 1
    assert a.get_process_info(b1).status is not process_status.waiting


def test_post_message_from_another_thread():
    a = pycvc.async_scheduler()
    pid = a.execute("(begin 1 2 3 4 5)")
    a.sync_step()
    assert a.receive_message(pid, "chan.x")
    worker = threading.Thread(target=a.post_message, args=("chan.x", "done"))
    worker.start()
    worker.join()
    a.drain_ingress()  # applied on this (the scheduler's) thread
    assert a.get_process_info(pid).status is not process_status.waiting
    assert a.sync_run()[pid] == 5


def test_run_releases_the_gil():
    # A Python thread keeps running while a scheduler runs a long program: run() and
    # sync_run() release the GIL around the C++ call.
    for sched, run, _ in _schedulers():
        sched.execute("(while 1 1)")
        ticks = []
        stop = threading.Event()

        def spin():
            while not stop.is_set():
                ticks.append(time.perf_counter())

        t = threading.Thread(target=spin)
        t.start()
        while not ticks:
            time.sleep(0.001)
        t0 = time.perf_counter()
        run(None, 0.3)
        t1 = time.perf_counter()
        stop.set()
        t.join()
        assert t1 - t0 >= 0.1
        # A call that held the GIL would stop the thread for its whole duration; it could
        # only run at the call's edges. So look in the middle third.
        third = (t1 - t0) / 3
        assert any(t0 + third < x < t1 - third for x in ticks)


# ── Exec ────────────────────────────────────────────────────────────────


def test_exec_exposes_its_scheduler():
    ex = pycvc.Exec(pycvc.make_app())
    sched = ex.scheduler()
    assert isinstance(sched, pycvc.async_scheduler)
    assert sched.parser_defaults().step_tokens == 1024  # host_parser_defaults, as set by Exec
    o = pycvc.parse_options()
    o.max_depth = 40
    ex.set_parser_defaults(o)
    assert sched.parser_defaults().max_depth == 40  # the same scheduler
    o.max_depth = 256
    sched.set_parser_defaults(o)
    assert ex.parser_defaults().max_depth == 256
    # A program submitted to it directly runs while run() drives the scheduler.
    pid = sched.execute("(+ 20 22)")
    assert ex.run("1") == "1"
    assert sched.get_result(pid) == 42
    # run() reaps its own processes, so only ours is left.
    assert [p.pid for p in sched.list_processes()] == [pid]
    # The scheduler keeps its Exec alive.
    del ex
    assert sched.process_count() == 1 and sched.reap(pid) and sched.process_count() == 0


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc state_exec scheduler parity tests: OK")
