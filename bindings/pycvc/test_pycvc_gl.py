"""pycvc_gl scene tests — Python builds a live 3D scene via cvcGL, headless.

Proves the generic binding chain end-to-end: geometry/volume built in the
`pycvc` module cross into the `pycvc_gl` module's Scene (SWIG %import) and drive
the cvcGL scene graph, with no Qt and no window.

Beyond the build smoke test, these exercise the *teardown* path that the C++
`cvcgl_remove` / `cvcgl_teardown` tests guard — the removeGraphics /
~SceneGraph use-after-free that was fixed by making scene nodes synchronous
(no per-node handler threads) and weak-guarding queued callbacks. The Python
facade reaches that path through Scene destruction: `del scene` -> `~Scene` ->
`sg_` shared_ptr reset -> `~SceneGraph`, which tears down every node (with
callbacks possibly still queued when a scene is dropped without a final pump).
Churning short-lived scenes with varying pump timing is the Python-level
analogue of cvcgl_teardown's Case A/B. All headless: no `show()`, no render.
"""

import gc

import pycvc

# Explicit app threaded through every op (no module-global).
app = pycvc.make_app()
import pycvc_gl


def _make_tri():
    g = pycvc.geometry(app)
    g.add_vertices([0, 0, 0, 10, 0, 0, 0, 10, 0])
    g.add_triangle(0, 1, 2)
    return g


def _make_field(n=3):
    v = pycvc.volume(app)
    v.set_float_grid([float(i) for i in range(n * n * n)], n, n, n, 0, 0, 0, 1, 1, 1)
    return v


def test_build_scene():
    scene = pycvc_gl.Scene()
    scene.add_geometry("tri", _make_tri())
    scene.add_volume("field", _make_field())
    scene.pump()  # re-pumpable, no Qt loop
    assert scene.has("tri")
    assert scene.has("field")
    assert not scene.has("nope")
    assert scene.num_graphics() == 2
    scene.pump()  # idempotent second pump
    assert scene.num_graphics() == 2
    print("  ok: scene builds mesh + scalar field, counts/has correct")


def test_scene_teardown_no_crash():
    # cvcgl_teardown Case A from Python: add, pump, then drop the scene with
    # (possibly) queued callbacks — must not fault.
    scene = pycvc_gl.Scene()
    scene.add_geometry("tri", _make_tri())
    scene.add_volume("field", _make_field())
    scene.pump()
    assert scene.num_graphics() == 2
    del scene  # ~Scene -> ~SceneGraph tears down every node
    gc.collect()
    # Build another scene afterward to prove the shared cvcGL context survived.
    scene2 = pycvc_gl.Scene()
    scene2.add_geometry("tri", _make_tri())
    assert scene2.num_graphics() == 1
    print("  ok: scene teardown after add+pump is crash-free (context survives)")


def test_scene_add_destroy_churn():
    # cvcgl_teardown Case B from Python: many short-lived scenes, varying pump
    # timing, dropped without a final drain. Deterministic (no handler threads),
    # so a few dozen iterations is a firm guard against the old UAF.
    tri = _make_tri()
    field = _make_field()
    for it in range(60):
        scene = pycvc_gl.Scene()
        for k in range(4):
            scene.add_geometry("g%d" % k, tri)
        scene.add_volume("field", field)
        if it % 3 == 0:
            scene.pump()  # sometimes pump, sometimes not
        assert scene.num_graphics() == 5
        assert scene.has("g0") and scene.has("field")
        del scene  # dropped with work possibly still queued
        if it % 10 == 0:
            gc.collect()
    gc.collect()
    print("  ok: 60 short-lived scenes churned add/destroy with no crash")


def test_scene_rebuild_same_names_across_scenes():
    # Distinct scenes reuse node names independently; teardown of one must not
    # affect the next (no shared/leaked node state across SceneGraph instances).
    for _ in range(10):
        s = pycvc_gl.Scene()
        s.add_geometry("shared", _make_tri())
        assert s.has("shared") and s.num_graphics() == 1
        del s
        gc.collect()
    print("  ok: node names reused cleanly across independent scenes")


if __name__ == "__main__":
    test_build_scene()
    test_scene_teardown_no_crash()
    test_scene_add_destroy_churn()
    test_scene_rebuild_same_names_across_scenes()
    print("pycvc_gl scene tests: OK")
