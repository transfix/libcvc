"""The app is threaded EXPLICITLY — there is no module-global, no attach/detach.

make_app() (a.k.a. pycvc.App) mints an independently-owned cvc::app; a host
would instead hand pycvc the shared_ptr it already runs under. Every op takes
the app: pycvc.volume(app), pycvc.state_set(app, ...), etc. Sharing ONE app
shares its whole state tree (state::instance(app) is per-app); two apps are
fully independent. This is the singleton-free contract.
"""

import gc
import threading

import pycvc


def test_no_module_global_api():
    # The singleton-era API is gone.
    assert not hasattr(pycvc, "attach")
    assert not hasattr(pycvc, "detach")
    assert not hasattr(pycvc, "ctx")
    # App creation is explicit.
    assert hasattr(pycvc, "make_app")
    assert pycvc.App is pycvc.make_app  # ergonomic alias


def test_one_app_is_one_shared_tree():
    app = pycvc.make_app()
    # Two independent code paths using the SAME app see the same state.
    pycvc.state_set(app, "shared.x", "hello")
    assert pycvc.state_get(app, "shared.x") == "hello"
    assert pycvc.state_has(app, "shared.x")

    # A volume built in this app carries it; state written via the app is
    # visible to anything else holding the same handle.
    v = pycvc.volume(app)
    pycvc.state_set(app, "shared.y", "world")
    assert pycvc.state_get(app, "shared.y") == "world"
    del v


def test_distinct_apps_are_isolated():
    a = pycvc.make_app()
    b = pycvc.make_app()
    pycvc.state_set(a, "k", "in-a")
    assert pycvc.state_has(a, "k")
    assert not pycvc.state_has(b, "k"), "a's state must not leak into b"
    pycvc.state_set(b, "k", "in-b")
    assert pycvc.state_get(a, "k") == "in-a"
    assert pycvc.state_get(b, "k") == "in-b"


def test_app_frees_cleanly():
    # No leaked shared_ptr<cvc::app>: dropping the last handle destroys the app.
    app = pycvc.make_app()
    pycvc.state_set(app, "t", "1")
    v = pycvc.volume(app)
    g = pycvc.geometry(app)
    del app, v, g
    gc.collect()  # must not warn "no destructor found" / leak


def test_objects_outliving_their_app_handle_variable():
    # Objects built from an app keep it alive via their own reference, so using
    # them after the local `app` name is gone is safe.
    def make():
        app = pycvc.make_app()
        return pycvc.volume(app)

    v = make()
    gc.collect()
    assert v.xdim() >= 0  # still usable


def _module_state_workload(tag):
    """What one app's Exec sees of what pycvc keeps on its module rather than in C statics: the
    IntEnum classes, the exception types, the dsl_error factory, the names C++ keeps (`who`), and
    the per-thread "an Exec's run() is active here" mark. Returns a transcript; tag makes each
    app's state its own."""
    t = []
    app = pycvc.make_app()
    ex = pycvc.Exec(app, policy=pycvc.scheduling_policy.priority_rr)
    pycvc.state_set(app, "mod.who", tag)
    t.append(("state", ex.run('(state-get "mod.who")')))
    t.append(("policy", ex.scheduler().policy() is pycvc.scheduling_policy.priority_rr))
    t.append(("status", pycvc.parse_process_status("killed") is pycvc.process_status.killed))
    t.append(("to_string", pycvc.to_string(pycvc.process_status.waiting)))
    try:
        ex.run("(begin\n  (set x")
        t.append(("parse_error", None))
    except pycvc.parse_error as e:
        t.append(("parse_error", e.line))
    try:
        ex.run('(raise (KeyError "%s" (dict "k" 1)))' % tag)
        t.append(("dsl_error", None))
    except pycvc.dsl_error as e:
        t.append(("dsl_error", e.dsl_class, tag in str(e)))
    # A `who` built here and dropped before the job runs: C++ keeps its own copy.
    who = "".join(["py-join-", tag])
    job = pycvc.make_concat_job([tag, 1], "-", who)
    del who
    gc.collect()
    t.append(("concat", pycvc.run_to_completion(job)))

    def apply_one(fn, arg):
        return pycvc.call_function(fn, [arg], "".join(["py-apply-", tag]))

    ex.register_fn("apply-one", apply_one)
    t.append(("call", ex.run('(apply-one string.length "%s")' % tag)))
    # From inside this Exec's run(), another Exec's priority call raises at once; once run()
    # returns, this thread runs none and the call goes through.
    other = pycvc.Exec(app)
    seen = []

    def call_other():
        try:
            other.get_priority(1)
        except RuntimeError as e:
            seen.append(str(e))
        return True

    ex.register_fn("call-other", call_other)
    t.append(("nested", ex.run("(call-other)"), list(seen)))
    t.append(("after", other.get_priority(1)))
    del ex, other, app
    gc.collect()
    return t


def test_apps_share_none_of_the_module_state():
    # Each app's transcript is the same alone, in either order, after another app is gone, and on
    # threads at once, and the two differ only by what is each app's own.
    a, b = "alpha", "beta"
    alone_a = _module_state_workload(a)
    alone_b = _module_state_workload(b)
    assert alone_a != alone_b
    assert ("state", a) in alone_a and ("concat", a + "-1") in alone_a
    assert ("call", str(len(a))) in alone_a
    assert ("parse_error", 2) in alone_a
    assert ("dsl_error", "KeyError", True) in alone_a
    assert ("nested", "#t", ["Exec.run is active on another thread"]) in alone_a
    assert ("after", None) in alone_a
    assert all(x[1] is True for x in alone_a if x[0] in ("policy", "status"))
    assert ("to_string", "waiting") in alone_a
    # The reverse order, and a later app made after the others are gone, start clean.
    assert _module_state_workload(b) == alone_b
    assert _module_state_workload(a) == alone_a
    # On threads at once.
    out = {}
    threads = [
        threading.Thread(target=lambda k=k: out.__setitem__(k, _module_state_workload(k)))
        for k in (a, b, a + "2", b + "2")
    ]
    for th in threads:
        th.start()
    for th in threads:
        th.join(60)
        assert not th.is_alive()
    assert out[a] == alone_a and out[b] == alone_b
    assert out[a + "2"] == _module_state_workload(a + "2")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc explicit-app tests: OK")
