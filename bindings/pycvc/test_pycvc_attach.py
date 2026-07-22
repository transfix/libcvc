"""Phase 0: prove the injected-app substrate shares one context, and that
detach() isolates.

pycvc routes every facade through ONE module-wide std::shared_ptr<cvc::app>
(pycvc_context.{h,cpp}). A host injects its app via attach(); Python and the
host then share that app's single state tree (cvc::state::instance(app) is
per-app). With nothing attached, ctx() lazily makes a standalone app.

These tests act as BOTH sides of that contract:
  * the module side, via pycvc.state_set / pycvc.state_get (which use ctx());
  * the "host" side, via pycvc.state_set_on / pycvc.state_get_on on a handle
    the test holds — the same object it attach()ed.

If attach() truly unified the context, a write on one side is visible on the
other. detach() must then drop that shared app so a fresh standalone tree does
NOT see the old writes.
"""

import pycvc


def _missing_raises(fn, *args):
    """True iff fn(*args) raises (SWIG surfaces the C++ out_of_range)."""
    try:
        fn(*args)
    except Exception:
        return True
    return False


# ── shared-app round trip: module write visible to host, and vice-versa ──


def test_attach_shares_context_both_directions():
    host = pycvc.make_app()
    pycvc.attach(host)
    try:
        # Module writes through ctx(); the host reads the SAME node.
        pycvc.state_set("phase0.x", "hello")
        assert pycvc.state_get_on(host, "phase0.x") == "hello", "host must see module's write"

        # Host writes on its handle; the module reads it through ctx().
        pycvc.state_set_on(host, "phase0.y", "world")
        assert pycvc.state_get("phase0.y") == "world", "module must see host's write"

        # Round-trips also agree read via the opposite accessor.
        assert pycvc.state_get("phase0.x") == "hello"
        assert pycvc.state_get_on(host, "phase0.y") == "world"
        print("  ok: attach() unifies context — writes cross the module/host boundary both ways")
    finally:
        pycvc.detach()


# ── detach isolates: fresh standalone tree cannot see the old writes ──


def test_detach_isolates():
    host = pycvc.make_app()
    pycvc.attach(host)
    pycvc.state_set("phase0.x", "hello")
    assert pycvc.state_get("phase0.x") == "hello"

    # Drop the shared app. The next ctx() lazily builds a fresh standalone app,
    # whose empty tree must not carry the attached app's nodes.
    pycvc.detach()
    assert _missing_raises(pycvc.state_get, "phase0.x"), (
        "after detach the fresh standalone tree must not see phase0.x"
    )

    # The detached host object still holds the value — it was the real owner.
    assert pycvc.state_get_on(host, "phase0.x") == "hello"
    print("  ok: detach() isolates — standalone tree is fresh; the host keeps its own state")


# ── distinct hosts are distinct trees ──


def test_distinct_hosts_are_distinct_trees():
    a = pycvc.make_app()
    b = pycvc.make_app()

    pycvc.attach(a)
    pycvc.state_set("phase0.who", "a")
    pycvc.attach(b)  # swap the module onto a different app
    try:
        # b's tree never saw the write that landed on a.
        assert _missing_raises(pycvc.state_get, "phase0.who"), "b's tree must be independent of a's"
        assert pycvc.state_get_on(a, "phase0.who") == "a"
        assert _missing_raises(pycvc.state_get_on, b, "phase0.who")
        print("  ok: separate make_app() handles are separate state trees")
    finally:
        pycvc.detach()


# ── standalone (no host) still works ──


def test_standalone_ctx_lazily_created():
    pycvc.detach()  # ensure no host attached
    pycvc.state_set("phase0.standalone", "up")
    assert pycvc.state_get("phase0.standalone") == "up"
    print("  ok: with no host attached, ctx() lazily creates a working standalone app")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
    print("pycvc attach/substrate tests: OK")
