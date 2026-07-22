"""The app is threaded EXPLICITLY — there is no module-global, no attach/detach.

make_app() (a.k.a. pycvc.App) mints an independently-owned cvc::app; a host
would instead hand pycvc the shared_ptr it already runs under. Every op takes
the app: pycvc.volume(app), pycvc.state_set(app, ...), etc. Sharing ONE app
shares its whole state tree (state::instance(app) is per-app); two apps are
fully independent. This is the singleton-free contract.
"""

import gc

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


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc explicit-app tests: OK")
