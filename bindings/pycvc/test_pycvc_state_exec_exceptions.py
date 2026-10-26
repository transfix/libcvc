"""DSL exceptions from Python (docs/STATE_EXEC_EXCEPTIONS.md 2.6 and the PY cases of section 6).

A registered Python function's exception is a PythonError in the DSL; a DSL exception that reaches
Python raises pycvc.dsl_error, whose type derives from pycvc.dsl_class(name) for each class in the
exception's chain; a PythonError that comes back unhandled re-raises the Python exception itself,
with the dsl_error as its __cause__. A process ended by an exception it didn't catch is `failed`.
The exceptions.h helpers, stackless_evaluator.raise_ and evaluator_state.uncaught are wrapped too.
"""

import pycvc
from pycvc import process_status


class CallerError(Exception):
    """An exception class of the caller's own, for PY-3."""


def _raises(exc, fn, *args):
    try:
        fn(*args)
    except exc as e:
        return e
    raise AssertionError(f"{fn.__name__}{args!r} did not raise {exc.__name__}")


def _divide(a, b):
    return a // b


def _exec():
    ex = pycvc.Exec(pycvc.make_app())
    ex.register_fn("py-divide", _divide)
    return ex


def test_python_error_is_catchable_in_the_dsl():
    # PY-1
    ex = _exec()
    got = ex.run(
        '(try (py-divide 1 0) (catch (PythonError e) (list (get-attr e "python_type") '
        '(get-attr e "origin") (get-attr e "function"))))'
    )
    assert got == '("ZeroDivisionError" "python" "py-divide")', got
    # A PythonError is a RuntimeError, so a catch of either class catches it.
    assert ex.run('(try (py-divide 1 0) (catch (RuntimeError e) "caught"))') == "caught"


def test_uncaught_python_error_reraises_the_original():
    # PY-2
    ex = _exec()
    e = _raises(ZeroDivisionError, ex.run, "(py-divide 1 0)")
    cause = e.__cause__
    assert isinstance(cause, pycvc.dsl_error), repr(cause)
    assert cause.dsl_class == "PythonError"
    assert cause.dsl_classes == ["PythonError", "RuntimeError", "Exception"]
    assert cause.value["python_type"] == "ZeroDivisionError"
    # The Exec still works afterward.
    assert ex.run("(+ 2 2)") == "4"


def test_a_callers_exception_class_comes_back():
    # PY-3: Python code catches its own exception types through a DSL program.
    ex = pycvc.Exec(pycvc.make_app())

    def fails(*_):
        raise CallerError("from the caller")

    ex.register_fn("fails", fails)
    e = _raises(CallerError, ex.run, "(begin (set x 1) (fails x))")
    assert str(e) == "from the caller"
    assert isinstance(e.__cause__, pycvc.dsl_error)
    assert e.__cause__.value["python_type"].endswith("CallerError")
    # Caught in the DSL it is a PythonError like any other.
    assert ex.run('(try (fails) (catch (PythonError e) "dsl caught it"))') == "dsl caught it"


def test_dsl_classes_derive_as_in_the_dsl():
    # PY-4
    ex = pycvc.Exec(pycvc.make_app())
    e = _raises(
        pycvc.dsl_class("ValueError"),
        ex.run,
        '(begin (defclass MyError ValueError) (raise (MyError "m")))',
    )
    assert isinstance(e, pycvc.dsl_error)
    assert isinstance(e, RuntimeError)
    assert isinstance(e, pycvc.dsl_class("Exception"))
    assert isinstance(e, pycvc.dsl_class("MyError"))
    assert not isinstance(e, pycvc.dsl_class("TypeError"))
    assert e.dsl_class == "MyError"
    assert str(e) == "MyError: m"
    assert issubclass(pycvc.dsl_class("NativeError"), pycvc.dsl_class("RuntimeError"))
    assert pycvc.dsl_class("ValueError") is pycvc.dsl_class("ValueError")


def test_dsl_error_fields():
    # PY-5
    ex = pycvc.Exec(pycvc.make_app())
    e = _raises(pycvc.dsl_error, ex.run, '(raise (KeyError "missing" (dict "k" 1)))')
    assert e.dsl_class == "KeyError"
    assert e.dsl_classes == ["KeyError", "Exception"]
    assert e.origin == "dsl"
    assert e.value["message"] == "missing"
    assert e.value["data"] == {"k": 1}
    assert isinstance(e.traceback, list) and e.traceback
    assert all(isinstance(line, str) for line in e.traceback)
    # A builtin's error has its class and origin "builtin".
    e = _raises(pycvc.dsl_class("IndexError"), ex.run, "(nth (list 1) 5)")
    assert e.origin == "builtin"


def test_on_complete_exception_is_unchanged():
    # PY-6: an exception raised in a Python on_complete is raised from the step, as before.
    sched = pycvc.async_scheduler()

    def done(_result):
        raise ValueError("on_complete failed")

    opts = pycvc.execute_options()
    opts.on_complete = done
    sched.execute("(+ 1 2)", opts)
    _raises(RuntimeError, sched.sync_run)


def test_evaluators_raise_dsl_error():
    for ev in (
        pycvc.stackless_evaluator(pycvc.builtins.make_default_environment()),
        pycvc.evaluator(pycvc.builtins.make_default_environment()),
    ):
        e = _raises(pycvc.dsl_class("ValueError"), ev.evaluate_script, '(raise (ValueError "v"))')
        assert e.dsl_classes == ["ValueError", "Exception"]
        e = _raises(pycvc.dsl_class("NameError"), ev.evaluate_script, "undefined-name")
        assert "undefined symbol" in str(e)
    # A Python function registered in an evaluator's environment, uncaught: the original again.
    env = pycvc.builtins.make_default_environment()
    pycvc.builtins.register_fn(env, "py-divide", _divide)
    ev = pycvc.stackless_evaluator(env)
    e = _raises(ZeroDivisionError, ev.evaluate_script, "(py-divide 1 0)")
    assert isinstance(e.__cause__, pycvc.dsl_error)


def test_schedulers_end_the_process_failed():
    # CX-1 and CX-2 from Python.
    for sched, run in ((pycvc.scheduler(), "run"), (pycvc.async_scheduler(), "sync_run")):
        pid = sched.execute('(raise (ValueError "bad"))')
        getattr(sched, run)()  # doesn't raise: the process failed
        info = sched.get_process_info(pid)
        assert info.status == process_status.failed, info
        assert info.exit_error == "ValueError: bad"
        assert info.exit_exception["message"] == "bad"
        assert sched.get_exit_exception(pid)["message"] == "bad"
        assert sched.get_exit_error(pid) == "ValueError: bad"
        e = _raises(pycvc.dsl_class("ValueError"), sched.raise_exit_exception, pid)
        assert e.value["message"] == "bad"
        stats = sched.get_stats()
        assert stats.failed == 1 and stats.killed == 0, stats
        good = sched.execute("(+ 1 2)")
        getattr(sched, run)()
        assert sched.get_exit_exception(good) is None
        sched.raise_exit_exception(good)  # not failed: nothing raised


def test_exception_helpers():
    names = pycvc.exception_class_names()
    assert names[0] == "Exception" and "ValueError" in names and "PythonError" in names
    assert pycvc.exception_class("Nope") is None
    cls = pycvc.exception_class("ValueError")
    assert cls["__name__"] == "ValueError"
    # A class is a class dict, a constructor, or a built-in class's name.
    assert pycvc.class_chain("ValueError") == ["ValueError", "Exception"]
    assert pycvc.class_chain(cls) == ["ValueError", "Exception"]
    assert pycvc.class_name(cls) == "ValueError"
    env = pycvc.environment()
    pycvc.register_exception_classes(env)
    ctor = env.lookup("KeyError")
    assert pycvc.class_name(pycvc.class_of_constructor(ctor)) == "KeyError"
    assert pycvc.class_name(ctor) == "KeyError"
    assert pycvc.class_of_constructor(5) is None
    _raises(KeyError, pycvc.class_chain, "Nope")
    _raises(TypeError, pycvc.class_chain, 5)

    e = pycvc.make_exception("ValueError", "bad", {"k": 1})
    assert (e["message"], e["data"], e["origin"]) == ("bad", {"k": 1}, "dsl")
    assert pycvc.make_exception(cls, "plain")["data"] is None
    # The copies match their classes by name chain.
    assert pycvc.is_exception(e) and not pycvc.is_exception({"message": "no class"})
    assert pycvc.is_instance(e, "Exception") and pycvc.is_instance(e, cls)
    assert not pycvc.is_instance(e, "TypeError")
    assert pycvc.class_name(pycvc.class_of_instance(e)) == "ValueError"
    assert pycvc.class_of_instance(5) is None
    assert pycvc.exception_message(e) == "bad"
    assert pycvc.exception_summary(e) == "ValueError: bad"
    assert pycvc.traceback_line(pycvc.parse("(+ 1 2)")) == "(+ 1 2)"
    assert pycvc.kTracebackFrames == 64
    assert pycvc.is_finished(process_status.failed) and pycvc.is_finished(process_status.killed)
    assert not pycvc.is_finished(process_status.waiting)


def test_raise_into_a_state():
    ev = pycvc.stackless_evaluator(pycvc.builtins.make_default_environment())
    st = ev.create_state(
        '(try (begin (set i 0) (while (< i 1000) (set i (+ i 1))) "done") '
        '(catch (KeyError e) (get-attr e "message")))'
    )
    ev.run(st, 30)
    assert not st.done and st.uncaught is None
    ev.raise_(st, pycvc.make_exception("KeyError", "from Python"))
    assert ev.run(st) == "from Python"
    # Nothing catches it: the state is done with it uncaught, and stepping raises it.
    st = ev.create_state("(begin (set i 0) (while (< i 1000) (set i (+ i 1))))")
    ev.run(st, 10)
    ev.raise_(st, pycvc.make_exception("ValueError", "stop"))
    assert st.done and st.uncaught["message"] == "stop"
    e = _raises(pycvc.dsl_class("ValueError"), ev.step, st)
    assert e.value["message"] == "stop"
    _raises(RuntimeError, ev.raise_, st, 5)  # not an exception (std::invalid_argument)


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc state_exec exception tests: OK")
