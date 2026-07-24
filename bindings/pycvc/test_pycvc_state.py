"""Direct state access on a given app's root + real PUSH callbacks.

Every op takes the app explicitly (no module-global): state_set/get/has/
children/remove(app, ...) and observer.watch(app). Change notification is a SWIG
director — subclass pycvc.state_observer, override on_changed(path), call
watch(app); C++ then calls the Python override synchronously on every mutation
of THAT app's tree, carrying the full dotted path (real push).
"""

import gc

import pycvc


# ── direct access: set / get / has / children / remove ──────────────────


def test_set_get_roundtrip():
    app = pycvc.make_app()
    pycvc.state_set(app, "app.title", "cvc")
    assert pycvc.state_get(app, "app.title") == "cvc"


def test_set_creates_intermediates_and_has():
    app = pycvc.make_app()
    assert not pycvc.state_has(app, "a.b.c")
    pycvc.state_set(app, "a.b.c", "deep")
    assert pycvc.state_has(app, "a.b.c")
    assert pycvc.state_has(app, "a.b")  # intermediate created
    assert pycvc.state_has(app, "a")
    assert not pycvc.state_has(app, "a.b.missing")


def test_get_missing_raises():
    app = pycvc.make_app()
    try:
        pycvc.state_get(app, "nope.not.here")
    except Exception:
        pass
    else:
        raise AssertionError("state_get on a missing path must raise")


def test_children_are_immediate_names():
    app = pycvc.make_app()
    pycvc.state_set(app, "root.x", "1")
    pycvc.state_set(app, "root.y", "2")
    pycvc.state_set(app, "root.y.deeper", "3")
    kids = sorted(pycvc.state_children(app, "root"))
    assert kids == ["x", "y"], kids  # immediate only, not "y.deeper"


def test_remove_is_idempotent_and_prunes_subtree():
    app = pycvc.make_app()
    pycvc.state_set(app, "gone.child", "v")
    assert pycvc.state_has(app, "gone.child")
    pycvc.state_remove(app, "gone")
    assert not pycvc.state_has(app, "gone")
    assert not pycvc.state_has(app, "gone.child")
    pycvc.state_remove(app, "gone")  # idempotent — no raise


# ── push callbacks via the state_observer director ──────────────────────


class _Recorder(pycvc.state_observer):
    """Python subclass overriding the C++ virtual on_changed()."""

    def __init__(self):
        super().__init__()
        self.paths = []

    def on_changed(self, path):
        self.paths.append(path)


def test_observer_push_receives_full_path():
    app = pycvc.make_app()
    obs = _Recorder()
    obs.watch(app)
    assert obs.watching()

    pycvc.state_set(app, "push.alpha", "1")
    pycvc.state_set(app, "push.beta.gamma", "2")

    assert "push.alpha" in obs.paths, obs.paths
    assert "push.beta.gamma" in obs.paths, obs.paths
    obs.unwatch()


def test_observer_only_sees_its_own_app():
    a, b = pycvc.make_app(), pycvc.make_app()
    obs = _Recorder()
    obs.watch(a)  # watching a only
    pycvc.state_set(b, "other.app", "x")  # write to b
    assert not any(p == "other.app" for p in obs.paths), "must not see another app's changes"
    pycvc.state_set(a, "mine", "y")
    assert any(p == "mine" for p in obs.paths)
    obs.unwatch()


def test_unwatch_stops_notifications():
    app = pycvc.make_app()
    obs = _Recorder()
    obs.watch(app)
    pycvc.state_set(app, "live.x", "1")
    assert any(p == "live.x" for p in obs.paths)

    obs.unwatch()
    assert not obs.watching()
    before = len(obs.paths)
    pycvc.state_set(app, "live.y", "2")
    assert len(obs.paths) == before  # no further callbacks after unwatch


def test_dead_observer_is_disconnected_safely():
    app = pycvc.make_app()
    obs = _Recorder()
    obs.watch(app)
    pycvc.state_set(app, "dead.x", "1")
    assert obs.paths

    # Drop the only reference; its scoped_connection must disconnect in the
    # C++ dtor so a later write does NOT call into a freed Python object.
    del obs
    gc.collect()
    pycvc.state_set(app, "dead.y", "2")  # must not crash / call a stale director
    pycvc.state_set(app, "dead.z", "3")


def test_multiple_observers_all_fire():
    app = pycvc.make_app()
    a, b = _Recorder(), _Recorder()
    a.watch(app)
    b.watch(app)
    pycvc.state_set(app, "multi.k", "v")
    assert any(p == "multi.k" for p in a.paths)
    assert any(p == "multi.k" for p in b.paths)
    a.unwatch()
    b.unwatch()


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc state + push-observer tests: OK")
