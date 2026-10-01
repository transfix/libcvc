"""pycvc_gl LOD surface -- LodGraphicsNode, SceneGraph.selectLOD / setLODEnabled /
lod_stats and make_view_params, driven from Python. Headless (the selection is
pure math; nothing here needs a GL context except the make_view_params check,
which skips itself when no offscreen context can be made).

Mirrors what cvcgl_lod_node proves in C++, from the Python side:

  * a LodGraphicsNode populated from a pycvc.mesh_pyramid -- built OR assembled
    by hand -- selects the finest rung near and the coarsest far, and draws
    exactly one rung;
  * progressive attach: setBase draws rung 0 at once, appendRungs keeps that
    very node (its metadata survives) when the pyramid was built from it;
  * setRungStyle with a Python callable reaches every current and future rung;
    a style that raises is re-raised from the call, with the ladder intact;
  * selectLOD's lod_stats (rung histogram, drawn vs full triangles, hidden
    nodes) and the setLODEnabled A/B switch;
  * the FEATURE-ON example, examples/lod_city.py: a synthetic tiled city
    built on a loader thread, baked to bytes, reloaded with open_verified and
    flown -- drawn triangles must drop at least 3x from the near pose to the
    far one.

Runs under pytest or as a plain script (ctest: pycvc_gl_lod).
"""

import math
import os
import sys

import pycvc
import pycvc_gl

app = pycvc.make_app()
_EXAMPLES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "examples")


def _box(cx=0.0, cy=0.0, cz=0.0, keep=12):
    """A unit box at (cx, cy, cz) keeping its first `keep` triangles."""
    g = pycvc.geometry(app)
    for k in range(8):
        g.add_vertex(cx + (0.5 if k & 1 else -0.5), cy + (0.5 if k & 2 else -0.5),
                     cz + (0.5 if k & 4 else -0.5))
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
             (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    g.add_triangles([i for f in faces[:keep] for i in f])
    return g


def _box_pyramid(errors, cx=0.0, cy=0.0, cz=0.0):
    """Hand-assembled ladder: rung k keeps 12 >> k triangles (12, 6, 3, 1) and
    carries errors[k], so selection is driven by the ladder alone."""
    pyr = pycvc.mesh_pyramid()
    pyr.rungs = [_box(cx, cy, cz, 12 >> k) for k in range(len(errors))]
    pyr.world_error_m = errors
    return pyr


def _visible_rungs(node):
    return node.drawn_rungs()  # rung ACTORS drawing (a switch flips the actor, not the node flag)


def _terrain(n, x0=0.0, y0=0.0, size=100.0):
    g = pycvc.geometry(app)
    for j in range(n + 1):
        for i in range(n + 1):
            x, y = x0 + size * i / n, y0 + size * j / n
            g.add_vertex(x, y, 6.0 + 4.0 * math.sin(0.21 * x) * math.cos(0.17 * y))
    tris = []
    for j in range(n):
        for i in range(n):
            a = j * (n + 1) + i
            tris += [a, a + 1, a + n + 2, a, a + n + 2, a + n + 1]
    g.add_triangles(tris)
    return g


def test_lod_node_selects_by_distance():
    sg = pycvc_gl.SceneGraph(app, "lod_select")
    node = sg.add_lod("box")
    assert isinstance(node, pycvc_gl.LodGraphicsNode)
    assert node.rungCount() == 0 and node.selectedRung() == -1 and node.activeRung() == -1
    node.setPyramid(_box_pyramid([0.0, 0.01, 0.1, 1.0]))
    assert node.rungCount() == 4
    assert node.rung_triangle_counts() == [12, 6, 3, 1]
    assert node.rung_errors() == [0.0, 0.01, 0.1, 1.0]
    assert node.rungTriangles(2) == 3 and node.rungError(3) == 1.0 and node.rungTriangles(9) == 0
    assert node.selectedRung() == -1 and node.activeRung() == 0  # rung 0 until the first select
    assert list(node.child_names()) == ["lod0", "lod1", "lod2", "lod3"]
    assert list(node.get_bounding_box()) == [-0.5, -0.5, -0.5, 0.5, 0.5, 0.5]
    # near -> finest, far -> coarsest, and exactly one rung drawn
    assert node.select(pycvc.view_at(0.0, 0.0, 2.0)) == 0
    assert node.selectedRung() == 0 and _visible_rungs(node) == [0]
    assert node.select(pycvc.view_at(0.0, 0.0, 1.0e5)) == 3
    assert node.selectedRung() == 3 and _visible_rungs(node) == [3]
    # setRung pins (clamped), select resumes from it
    assert node.setRung(1) == 1 and _visible_rungs(node) == [1]
    assert node.setRung(99) == 3
    # the typed lookups find it, with an app keep-alive
    assert isinstance(sg.getGraphics("box"), pycvc_gl.LodGraphicsNode)
    assert sg.lod_node("box").rungCount() == 4 and sg.lod_node("nope") is None
    assert node.rung(0)._pycvc_app is app and node.rung(7) is None
    # a directly constructed node joins the scene through add_node
    own = pycvc_gl.LodGraphicsNode(app, "lod_select.mine", "mine")
    sg.add_node("mine", own)
    own.setPyramid(_box_pyramid([0.0, 0.5], cx=10.0))
    assert sg.lod_node("mine").rungCount() == 2
    print("  ok: LodGraphicsNode setPyramid / select / setRung / typed lookup")


def test_progressive_attach_keeps_rung0():
    sg = pycvc_gl.SceneGraph(app, "lod_progressive")
    node = sg.add_lod("ground")
    g = _terrain(24)
    node.setBase(g)
    assert node.rungCount() == 1 and node.activeRung() == 0
    node.rung(0).set_metadata("uploaded", True)  # stands in for GPU state on rung 0
    pyr = pycvc.build_mesh_pyramid(g)
    assert len(pyr) > 1
    assert node.appendRungs(pyr) is True  # rung 0 kept
    assert node.rungCount() == len(pyr) and node.rung(0).get_metadata("uploaded") is True
    assert node.rung_triangle_counts() == pyr.rung_triangles()
    # a pyramid of DIFFERENT content falls back to setPyramid (rung 0 rebuilt)
    other = pycvc.build_mesh_pyramid(_terrain(24, x0=500.0))
    assert node.appendRungs(other) is False
    assert node.rung(0).get_metadata("uploaded") is None
    assert node.appendRungs(pycvc.mesh_pyramid()) is False  # empty: no change
    assert node.rungCount() == len(other)
    print("  ok: setBase + appendRungs keeps rung 0 (%d rungs)" % node.rungCount())


def test_rung_style_callable():
    sg = pycvc_gl.SceneGraph(app, "lod_style")
    node = sg.add_lod("styled")
    node.setPyramid(_box_pyramid([0.0, 0.1, 1.0]))
    styled = []

    def style(n):
        assert isinstance(n, pycvc_gl.GeometryNode)
        styled.append(n.getName())
        n.setColor(1.0, 0.5, 0.25)
        n.set_metadata("style", n.getName())

    node.setRungStyle(style)  # every existing rung, now
    assert styled == ["lod0", "lod1", "lod2"]
    node.setPyramid(_box_pyramid([0.0, 0.1, 1.0, 2.0]))  # and every new one
    assert styled[3:] == ["lod0", "lod1", "lod2", "lod3"]
    assert [node.rung(k).get_metadata("style") for k in range(4)] == ["lod0", "lod1", "lod2", "lod3"]
    node.setRungStyle(None)  # stop styling new rungs
    node.setPyramid(_box_pyramid([0.0, 0.1]))
    assert len(styled) == 7 and node.rung(0).get_metadata("style") is None

    class StyleError(Exception):
        pass

    def bad(n):
        if n.getName() == "lod1":
            raise StyleError("no style for " + n.getName())

    try:
        node.setRungStyle(bad)
        raise AssertionError("setRungStyle should have re-raised the style's exception")
    except StyleError as e:
        assert str(e) == "no style for lod1"
    try:
        node.setPyramid(_box_pyramid([0.0, 0.1, 1.0]))
        raise AssertionError("setPyramid should have re-raised the style's exception")
    except StyleError:
        pass
    # the ladder is intact: every rung built, every rung a child, one drawn
    assert node.rungCount() == 3 and list(node.child_names()) == ["lod0", "lod1", "lod2"]
    assert node.select(pycvc.view_at(0.0, 0.0, 1.0e5)) == 2 and _visible_rungs(node) == [2]
    try:
        node.setRungStyle(42)
        raise AssertionError("a non-callable style should raise TypeError")
    except TypeError:
        pass
    print("  ok: setRungStyle(callable) styles current + future rungs; errors re-raised")


def test_select_lod_stats_and_switch():
    sg = pycvc_gl.SceneGraph(app, "lod_stats_city")
    sg.add_group("city")
    errors = [0.0, 1.0, 2.0, 4.0]  # switch radii ~480 m, ~970 m, ~1.9 km (balanced)
    n = 0
    for j in range(4):
        for i in range(4):
            node = sg.add_child_lod("city", "b%d" % n)
            node.setPyramid(_box_pyramid(errors, cx=100.0 * i, cy=100.0 * j))
            n += 1
    st = pycvc_gl.lod_stats()
    # near, inside the city: everything close enough for rung 0
    near = pycvc.view_at(150.0, 150.0, 10.0)
    assert sg.selectLOD(near, st) == 0  # rung 0 was already drawing: not a change
    assert st.nodes == 16 and st.hidden == 0 and st.rung_nodes == [16]
    assert st.drawn_tris == st.full_tris == 16 * 12 and st.saving() == 1.0
    # far: every node on the coarsest rung
    far = pycvc.view_at(150.0, -1.0e5, 1.0e4)
    assert sg.selectLOD(far, st) == 16 and st.changes == 16
    assert st.rung_nodes == [0, 0, 0, 16] and st.drawn_tris == 16 and st.full_tris == 192
    assert math.isclose(st.saving(), 12.0)
    # select_lod_stats: the same pass, a fresh stats object back
    st2 = sg.select_lod_stats(far)
    assert st2.changes == 0 and st2.rung_nodes == [0, 0, 0, 16]
    assert "drawn_tris=16" in repr(st2)
    # a hidden node is still selected, counted as hidden, and draws nothing
    sg.lod_node("b0").setVisible(False)
    st3 = sg.select_lod_stats(far)
    assert st3.nodes == 16 and st3.hidden == 1 and st3.drawn_tris == 15
    assert _visible_rungs(sg.lod_node("b0")) == []
    sg.lod_node("b0").setVisible(True)
    assert _visible_rungs(sg.lod_node("b0")) == [3]  # back on the right rung at once
    # the A/B switch
    assert sg.lodEnabled()
    sg.setLODEnabled(False)
    assert not sg.lodEnabled()
    off = sg.select_lod_stats(far)
    assert off.changes == 16 and off.rung_nodes == [16] and off.drawn_tris == off.full_tris
    sg.setLODEnabled(True)
    assert sg.select_lod_stats(far).rung_nodes == [0, 0, 0, 16]
    # selectLOD with no stats object
    assert sg.selectLOD(near) == 16 and sg.selectLOD(near, None) == 0
    print("  ok: selectLOD / lod_stats / hidden nodes / setLODEnabled")


def test_make_view_params_from_a_live_camera():
    sg = pycvc_gl.SceneGraph(app, "lod_view")
    node = sg.add_lod("box")
    node.setPyramid(_box_pyramid([0.0, 0.01, 0.1, 1.0]))
    base = pycvc.preset_view("aggressive")
    try:
        view = pycvc_gl.SceneRenderer(sg, 320, 200, True, "lodview")
        view.setCamera(0.0, -50.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 30.0, 0.1, 1e4)
        view.render()
    except Exception as e:  # noqa: BLE001 -- no offscreen GL context here
        print("  skip: make_view_params (no offscreen GL: %s)" % e)
        return
    v = view.make_view_params(base)
    assert v.viewport_h_px == view.frameHeight() == 200
    assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(v.eye, (0.0, -50.0, 10.0)))
    assert math.isclose(v.tan_half_fov, math.tan(math.radians(15.0)), rel_tol=1e-9)
    assert v.desired_pixel_error == base.desired_pixel_error and v.ortho_px_per_m == 0.0
    # the free function on the renderer handle gives the same view
    v2 = pycvc_gl.make_view_params(view.renderer(), base)
    assert v2.eye == v.eye and v2.viewport_h_px == v.viewport_h_px
    st = sg.select_lod_stats(v)
    assert st.nodes == 1 and sum(st.rung_nodes) == 1
    view.close()
    print("  ok: SceneRenderer.make_view_params -> eye %s, %d px, rung %d"
          % (v.eye, v.viewport_h_px, node.selectedRung()))


def test_lod_city_example():
    sys.path.insert(0, _EXAMPLES)
    try:
        import lod_city
    finally:
        sys.path.remove(_EXAMPLES)
    r = lod_city.run(blocks=4, terrain_n=24, workers=2, verbose=False)
    assert r["tiles"] == 16 and r["rung0_kept"] == 16  # progressive attach kept every rung 0
    assert r["frames_during_build"] >= 1 and len(r["sha256"]) == 64 and r["blob_bytes"] > 0
    poses = r["poses"]
    near, far = poses[0], poses[-1]
    assert near["nodes"] == far["nodes"] == 16
    assert near["full_tris"] == far["full_tris"] == r["source_tris"]
    # FEATURE ON: drawn triangles drop at least 3x from the near pose to the far one
    assert near["drawn_tris"] >= 3 * far["drawn_tris"], (near["drawn_tris"], far["drawn_tris"])
    # and never rise as the camera pulls away
    drawn = [p["drawn_tris"] for p in poses]
    assert all(a >= b for a, b in zip(drawn, drawn[1:])), drawn
    assert r["lod_off"]["drawn_tris"] == r["lod_off"]["full_tris"]
    print("  ok: lod_city: %s drawn along the path (%.1fx near -> far)"
          % (drawn, near["drawn_tris"] / far["drawn_tris"]))


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            print(name)
            fn()
    print("pycvc_gl LOD tests: OK")
