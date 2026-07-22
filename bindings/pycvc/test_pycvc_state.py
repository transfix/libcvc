"""Phase 3: direct state access on the shared root + real PUSH callbacks.

pycvc exposes the injected app's state tree directly (no facade State object):
state_set/state_get (Phase 0) + state_has/state_children/state_remove. Change
notification is a SWIG director — subclass pycvc.state_observer, override
on_changed(path), call watch(); C++ then calls the Python override synchronously
on every mutation, carrying the full dotted path. This REPLACES the old
record-and-poll (#132) with true push.
"""

import gc

import pycvc


def _fresh():
    """Detach any host app so each test runs on its own standalone tree."""
    pycvc.detach()


# ── direct access: set / get / has / children / remove ──────────────────


def test_set_get_roundtrip():
    _fresh()
    pycvc.state_set("app.title", "cvc")
    assert pycvc.state_get("app.title") == "cvc"


def test_set_creates_intermediates_and_has():
    _fresh()
    assert not pycvc.state_has("a.b.c")
    pycvc.state_set("a.b.c", "deep")
    assert pycvc.state_has("a.b.c")
    assert pycvc.state_has("a.b")  # intermediate created
    assert pycvc.state_has("a")
    assert not pycvc.state_has("a.b.missing")


def test_get_missing_raises():
    _fresh()
    try:
        pycvc.state_get("nope.not.here")
    except Exception:
        pass
    else:
        raise AssertionError("state_get on a missing path must raise")


def test_children_are_immediate_names():
    _fresh()
    pycvc.state_set("root.x", "1")
    pycvc.state_set("root.y", "2")
    pycvc.state_set("root.y.deeper", "3")
    kids = sorted(pycvc.state_children("root"))
    assert kids == ["x", "y"], kids  # immediate only, not "y.deeper"


def test_remove_is_idempotent_and_prunes_subtree():
    _fresh()
    pycvc.state_set("gone.child", "v")
    assert pycvc.state_has("gone.child")
    pycvc.state_remove("gone")
    assert not pycvc.state_has("gone")
    assert not pycvc.state_has("gone.child")
    pycvc.state_remove("gone")  # idempotent — no raise


# ── push callbacks via the state_observer director ──────────────────────


class _Recorder(pycvc.state_observer):
    """Python subclass overriding the C++ virtual on_changed()."""

    def __init__(self):
        super().__init__()
        self.paths = []

    def on_changed(self, path):
        self.paths.append(path)


def test_observer_push_receives_full_path():
    _fresh()
    obs = _Recorder()
    obs.watch()
    assert obs.watching()

    pycvc.state_set("push.alpha", "1")
    pycvc.state_set("push.beta.gamma", "2")

    # C++ called the Python override synchronously, with the full dotted path.
    assert "push.alpha" in obs.paths, obs.paths
    assert "push.beta.gamma" in obs.paths, obs.paths
    obs.unwatch()


def test_unwatch_stops_notifications():
    _fresh()
    obs = _Recorder()
    obs.watch()
    pycvc.state_set("live.x", "1")
    assert any(p == "live.x" for p in obs.paths)

    obs.unwatch()
    assert not obs.watching()
    before = len(obs.paths)
    pycvc.state_set("live.y", "2")
    assert len(obs.paths) == before  # no further callbacks after unwatch


def test_dead_observer_is_disconnected_safely():
    _fresh()
    obs = _Recorder()
    obs.watch()
    pycvc.state_set("dead.x", "1")
    assert obs.paths

    # Drop the only reference; its scoped_connection must disconnect in the
    # C++ dtor so a later write does NOT call into a freed Python object.
    del obs
    gc.collect()
    pycvc.state_set("dead.y", "2")  # must not crash / call a stale director
    pycvc.state_set("dead.z", "3")


def test_multiple_observers_all_fire():
    _fresh()
    a, b = _Recorder(), _Recorder()
    a.watch()
    b.watch()
    pycvc.state_set("multi.k", "v")
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
