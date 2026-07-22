"""Robust, deterministic tests for pycvc.State — the reactive state-tree facade.

pycvc.State wraps cvc::state, the process-wide reactive tree of dotted-path
nodes (each holding a string value). The tree is a PROCESS singleton, so every
State() instance shares the same root; each test therefore uses its own unique
path prefix to stay independent. The change record (poll_changes), by contrast,
is per-facade: a fresh State() starts with an empty record and only observes
nodes that same facade has written.

Threading model (inherited from #126): the bindings add no locking of their
own, but cvc::state is internally mutex-protected per node, so the safe,
asserted invariant is that N threads writing DISTINCT paths all land correctly
and the tree stays consistent. Coordination uses threading.Barrier (never
sleeps) so the threads genuinely overlap.
"""

import threading

import pycvc


# ── set / get round trip ────────────────────────────────────────────


def test_set_get_roundtrip():
    st = pycvc.State()
    st.set("rt.a", "1")
    st.set("rt.b", "two")
    st.set("rt.nested.deep.leaf", "42")
    assert st.get("rt.a") == "1"
    assert st.get("rt.b") == "two"
    assert st.get("rt.nested.deep.leaf") == "42"
    print("  ok: set/get round-trips for flat and nested dotted paths")


def test_overwrite_and_reread():
    st = pycvc.State()
    st.set("ow.x", "first")
    assert st.get("ow.x") == "first"
    st.set("ow.x", "second")
    assert st.get("ow.x") == "second"
    st.set("ow.x", "third")
    assert st.get("ow.x") == "third"
    print("  ok: overwrite + re-read reflects the latest value")


def test_get_missing_raises():
    st = pycvc.State()
    try:
        st.get("does.not.exist.anywhere")
    except Exception:  # SWIG surfaces the C++ out_of_range as a Python exception
        print("  ok: get() on a missing path raises")
        return
    raise AssertionError("expected get() on a missing path to raise")


# ── has / children ──────────────────────────────────────────────────


def test_has():
    st = pycvc.State()
    st.set("hs.parent.child", "v")
    assert st.has("hs.parent.child") is True
    assert st.has("hs.parent") is True  # intermediate node was created
    assert st.has("hs") is True
    assert st.has("hs.parent.sibling") is False
    assert st.has("hs.absent") is False
    print("  ok: has() reports existence; set() creates intermediate nodes")


def test_children_immediate_only():
    st = pycvc.State()
    st.set("ch.a", "1")
    st.set("ch.b", "2")
    st.set("ch.c.deep", "3")  # 'c' is an immediate child; 'deep' is a grandchild
    kids = set(st.children("ch"))
    assert kids == {"a", "b", "c"}, kids
    # Grandchild is reachable one level down, not flattened into ch's children.
    assert set(st.children("ch.c")) == {"deep"}
    # Missing path -> empty list (not an error).
    assert list(st.children("ch.absent")) == []
    print("  ok: children() returns immediate child leaf-names only")


# ── remove ──────────────────────────────────────────────────────────


def test_remove():
    st = pycvc.State()
    st.set("rm.keep", "k")
    st.set("rm.gone.sub", "g")
    assert st.has("rm.gone") and st.has("rm.gone.sub")
    st.remove("rm.gone")
    assert st.has("rm.gone") is False
    assert st.has("rm.gone.sub") is False
    assert st.has("rm.keep") is True  # sibling untouched
    # Idempotent: removing an absent path is a no-op.
    st.remove("rm.gone")
    # Re-create at the same path and confirm it is watched again.
    st.set("rm.gone.sub", "again")
    assert st.get("rm.gone.sub") == "again"
    print("  ok: remove() deletes a subtree, is idempotent, and re-create works")


# ── poll_changes (record-and-poll change observation) ───────────────


def test_poll_changes_exact():
    st = pycvc.State()
    st.set("pc.x", "1")
    st.set("pc.y", "1")
    st.set("pc.z", "1")
    changed = set(st.poll_changes())
    assert changed == {"pc.x", "pc.y", "pc.z"}, changed
    # Draining clears the record.
    assert list(st.poll_changes()) == []
    # A no-op write (same value) does NOT record a change...
    st.set("pc.x", "1")
    assert list(st.poll_changes()) == []
    # ...but a real change does, and only for the path that changed.
    st.set("pc.y", "2")
    assert set(st.poll_changes()) == {"pc.y"}
    print("  ok: poll_changes() reports exactly the paths that changed, then clears")


def test_poll_changes_is_per_facade():
    a = pycvc.State()
    b = pycvc.State()
    a.set("pf.only_a", "1")
    # b never wrote pf.only_a, so it is not subscribed and sees nothing.
    assert set(a.poll_changes()) == {"pf.only_a"}
    assert list(b.poll_changes()) == []
    print("  ok: each facade's change record is independent (per-facade subscription)")


# ── threaded: N threads write distinct paths ────────────────────────


def test_threaded_distinct_paths():
    st = pycvc.State()
    n = 8
    writes_per_thread = 50
    barrier = threading.Barrier(n)
    errors = []

    def worker(tid):
        try:
            barrier.wait()  # all threads start together, genuinely overlapping
            for i in range(writes_per_thread):
                path = "th.%d.item.%d" % (tid, i)
                st.set(path, "%d-%d" % (tid, i))
        except Exception as e:  # noqa: BLE001
            errors.append(repr(e))

    threads = [threading.Thread(target=worker, args=(t,)) for t in range(n)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert not errors, errors
    # Invariant 1: every distinct path holds exactly the value its writer set.
    for tid in range(n):
        for i in range(writes_per_thread):
            assert st.get("th.%d.item.%d" % (tid, i)) == "%d-%d" % (tid, i)
    # Invariant 2: each thread's namespace has exactly writes_per_thread items.
    for tid in range(n):
        assert len(st.children("th.%d.item" % tid)) == writes_per_thread
    # Invariant 3: poll_changes recorded exactly the full set of written paths.
    expected = {
        "th.%d.item.%d" % (tid, i) for tid in range(n) for i in range(writes_per_thread)
    }
    assert set(st.poll_changes()) == expected
    print("  ok: %d threads wrote %d distinct paths each; tree consistent, changes exact"
          % (n, writes_per_thread))


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
    print("pycvc.State tests: OK")
