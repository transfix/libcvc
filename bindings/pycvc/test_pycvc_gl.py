"""pycvc_gl scene tests — Python drives a live 3D scene via the DIRECTLY-WRAPPED
cvcGL objects, headless.

pycvc_gl wraps the REAL cvcGL classes (no facade): SceneGraph and the
SceneNode -> GraphicsNode -> {GeometryNode, VolumeNode} hierarchy, held by
shared_ptr. geometry/volume built in the `pycvc` module cross in via SWIG
%import; addGraphics() returns the LIVE node, and node.setPosition() /
setColor() / setGeometry() mutate it IN PLACE — the animation path is
"move coordinates," never destroy+recreate.

Beyond the build smoke test, these exercise the *teardown* path the C++
`cvcgl_remove` / `cvcgl_teardown` tests guard — the removeGraphics /
~SceneGraph use-after-free fixed by making scene nodes synchronous (no per-node
handler threads) and weak-guarding queued callbacks. Churning short-lived
SceneGraphs with varying pump timing is the Python-level analogue of
cvcgl_teardown's Case A/B. All headless: no show(), no render.
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
    g.set_colors([1.0, 0.0, 0.0] * 3)
    return g


def _make_field(n=3):
    v = pycvc.volume(app)
    v.set_float_grid([float(i) for i in range(n * n * n)], n, n, n, 0, 0, 0, 1, 1, 1)
    return v


def test_build_scene():
    sg = pycvc_gl.SceneGraph(app)
    # addGraphics returns the live node (GraphicsNode for geometry, VolumeNode
    # for a volume).
    tri = sg.addGraphics("tri", _make_tri())
    fld = sg.addGraphics("field", _make_field())
    assert tri is not None and fld is not None
    sg.processEvents()  # re-pumpable, no Qt loop
    assert sg.hasGraphics("tri")
    assert sg.hasGraphics("field")
    assert not sg.hasGraphics("nope")
    assert sg.num_graphics() == 2
    sg.processEvents()  # idempotent second pump
    assert sg.num_graphics() == 2
    print("  ok: scene builds mesh + scalar field, counts/has correct")


def test_update_ops_no_rebuild():
    # THE point of the direct wrap: get the node once, mutate it in place. The
    # node identity and the scene node-count are unchanged across many moves.
    sg = pycvc_gl.SceneGraph(app)
    node = sg.addGraphics("agent0", _make_tri())
    assert sg.num_graphics() == 1

    # Transform updates (move / scale / rotate / full matrix / reset).
    for k in range(50):
        node.setPosition(float(k), 2.0 * k, 0.5 * k)  # walk it
    node.setScale(2.0, 2.0, 2.0)
    node.setRotation(0.0, 0.0, 45.0)
    node.setTransform([1, 0, 0, 3, 0, 1, 0, 4, 0, 0, 1, 5, 0, 0, 0, 1])  # row-major 4x4
    node.resetTransform()
    # Same live node comes back from getGraphics; still ONE node (no rebuild).
    assert sg.getGraphics("agent0") is not None
    assert sg.num_graphics() == 1

    # Typed accessor -> concrete GeometryNode: material + in-place DATA update.
    gn = sg.geometry_node("agent0")
    assert gn is not None
    gn.setColor(0.2, 0.9, 0.3)
    gn.setOpacity(0.8)
    gn.setGeometry(_make_tri())  # replace data in place, still one node
    assert sg.num_graphics() == 1

    # Bad transform length is rejected.
    try:
        node.setTransform([1, 2, 3])
    except Exception:
        pass
    else:
        raise AssertionError("setTransform must require 16 elements")

    # A typed accessor for the wrong type / missing name yields None.
    assert sg.volume_node("agent0") is None
    assert sg.geometry_node("nope") is None
    print("  ok: in-place update ops (move/scale/rotate/transform/color/data) — no rebuild")


def test_remove():
    sg = pycvc_gl.SceneGraph(app)
    sg.addGraphics("a", _make_tri())
    sg.addGraphics("b", _make_tri())
    assert sg.num_graphics() == 2
    sg.removeGraphics("a")
    assert not sg.hasGraphics("a")
    assert sg.hasGraphics("b")
    assert sg.num_graphics() == 1
    print("  ok: removeGraphics drops the node, others survive")


def test_scene_teardown_no_crash():
    # cvcgl_teardown Case A from Python: add, pump, then drop the scene with
    # (possibly) queued callbacks — must not fault.
    sg = pycvc_gl.SceneGraph(app)
    sg.addGraphics("tri", _make_tri())
    sg.addGraphics("field", _make_field())
    sg.processEvents()
    assert sg.num_graphics() == 2
    del sg  # ~SceneGraph tears down every node
    gc.collect()
    # Build another scene afterward to prove the shared cvcGL context survived.
    sg2 = pycvc_gl.SceneGraph(app)
    sg2.addGraphics("tri", _make_tri())
    assert sg2.num_graphics() == 1
    print("  ok: scene teardown after add+pump is crash-free (context survives)")


def test_scene_add_destroy_churn():
    # cvcgl_teardown Case B from Python: many short-lived scenes, varying pump
    # timing, dropped without a final drain. Deterministic (no handler threads),
    # so a few dozen iterations is a firm guard against the old UAF.
    tri = _make_tri()
    field = _make_field()
    for it in range(60):
        sg = pycvc_gl.SceneGraph(app)
        for k in range(4):
            sg.addGraphics("g%d" % k, tri)
        sg.addGraphics("field", field)
        if it % 3 == 0:
            sg.processEvents()  # sometimes pump, sometimes not
        assert sg.num_graphics() == 5
        assert sg.hasGraphics("g0") and sg.hasGraphics("field")
        del sg  # dropped with work possibly still queued
        if it % 10 == 0:
            gc.collect()
    gc.collect()
    print("  ok: 60 short-lived scenes churned add/destroy with no crash")


def test_held_node_outlives_scene():
    # A node proxy kept after its scene is dropped must not fault at teardown:
    # the app keep-alive is propagated onto returned nodes, so ~SceneNode never
    # locks a destroyed state mutex. (This is the shutdown-ordering hazard the
    # direct wrap introduced and the %pythonappend keep-alive closes.)
    sg = pycvc_gl.SceneGraph(app)
    held = sg.addGraphics("keep", _make_tri())
    sg.removeGraphics("keep")  # scene no longer owns it; only `held` does
    del sg
    gc.collect()
    held.setPosition(1.0, 2.0, 3.0)  # still usable
    del held
    gc.collect()
    print("  ok: a held node proxy outlives its scene and tears down cleanly")


def test_director_python_node_type():
    # Directors: Python can SUBCLASS a wrapped C++ node and C++ dispatches its
    # virtuals into the Python override (cross-language polymorphism). This is
    # what makes Python-defined scene node types possible — e.g. a getProp()
    # override returning a vtkmodules-built prop.
    class TaggedNode(pycvc_gl.GeometryNode):
        def __init__(self, a, path, name):
            super().__init__(a, path, name)
            self.update_calls = 0

        def update(self):  # public virtual override
            self.update_calls += 1
            pycvc_gl.GeometryNode.update(self)  # chain to the C++ base

    node = TaggedNode(app, "cvcgl.graphics.children.tagged", "tagged")
    assert node.update_calls == 0
    # C++ invokes update() through a base-class handle; must land in Python.
    pycvc_gl.poke_update(node)
    pycvc_gl.poke_update(node)
    assert node.update_calls == 2, "director did not dispatch to the Python override"
    print("  ok: director — C++ dispatches a virtual into a Python-defined node type")


def test_group_nodes_compose_transforms():
    """A pure grouping node: no geometry, just a transform its children inherit.

    This is what add_child_geometry/add_child_volume alone could not express —
    every node had to carry geometry, so a caller with a multi-step local
    transform had to flatten it in Python instead of letting the graph compose
    it. Here the graph does the composing.
    """
    sg = pycvc_gl.SceneGraph(app)
    arm = sg.add_group("arm")
    assert arm is not None
    forearm = sg.add_child_group("arm", "forearm")
    hand = sg.add_child_geometry("forearm", "hand", _make_tri())
    assert sg.hasGraphics("arm") and sg.hasGraphics("forearm") and sg.hasGraphics("hand")

    # Translate along the chain; the leaf's WORLD transform must accumulate all
    # three, which is the whole point of grouping.
    arm.setPosition(10.0, 0.0, 0.0)
    forearm.setPosition(0.0, 5.0, 0.0)
    hand.setPosition(0.0, 0.0, 2.0)
    sg.processEvents()

    w = hand.get_world_transform()  # row-major 4x4
    assert abs(w[3] - 10.0) < 1e-9, w[3]
    assert abs(w[7] - 5.0) < 1e-9, w[7]
    assert abs(w[11] - 2.0) < 1e-9, w[11]

    # Moving the group moves the whole subtree, without touching the leaf.
    arm.setPosition(-4.0, 0.0, 0.0)
    sg.processEvents()
    w = hand.get_world_transform()
    assert abs(w[3] + 4.0) < 1e-9, w[3]
    assert abs(w[7] - 5.0) < 1e-9

    assert "forearm" in arm.child_names()
    assert "hand" in forearm.child_names()
    print("  ok: group nodes carry transforms their children inherit")


if __name__ == "__main__":
    test_build_scene()
    test_update_ops_no_rebuild()
    test_remove()
    test_scene_teardown_no_crash()
    test_scene_add_destroy_churn()
    test_held_node_outlives_scene()
    test_director_python_node_type()
    test_group_nodes_compose_transforms()
    print("pycvc_gl scene tests: OK")
