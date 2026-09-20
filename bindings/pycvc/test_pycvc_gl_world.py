"""pycvc_gl world-units / scene-node coverage — the surface added alongside
cvc::world_units, headless.

Groups four additions to the wrapped cvcGL surface:

  1. GraphicsNode real-world geometry: local_to_world / world_to_local resolve a
     point through the FULL chain of local transforms, and local_point_to_real /
     real_dimensions / get_world_bounding_box project that into a world_units
     regime. This is the "reliable dimensions through the transform chain" the
     digital-twin work needs, driven from Python.
  2. The built-in reference nodes: getGridNode()/getAxisNode() are now wrapped
     (GridNode/AxisNode), so a script can size/colour the grid and the axis.
  3. Lights: addLight() returns a wrapped LightNode (kind as a string, target/
     colour/world-position as tuples), reachable again via light_node()/
     getGraphics().
  4. VolRenNode + the headless volren raycaster value types (camera eye/focal/up
     and colour arrays cross as tuples via the pycvc_volren.i typemaps).

All headless: nodes are built and mutated, never shown/rendered. Anything that
needs a live GL context (SceneRenderer.pick_world) is asserted at the binding
level only, with a note.
"""

import math

import pycvc
import pycvc_gl

app = pycvc.make_app()


def _make_tri():
    g = pycvc.geometry(app)
    # A right triangle with a corner at the LOCAL origin, so local_to_world of
    # [0,0,0] lands exactly on the node's world position.
    g.add_vertices([0, 0, 0, 10, 0, 0, 0, 10, 0])
    g.add_triangle(0, 1, 2)
    g.set_colors([1.0, 0.0, 0.0] * 3)
    return g


def _make_field(n=4):
    v = pycvc.volume(app)
    v.set_float_grid([float(i) for i in range(n * n * n)], n, n, n, 0, 0, 0, 1, 1, 1)
    return v


# ── 1. GraphicsNode: dimensions through the chain of local transforms ────────


def test_local_to_world_composes_the_transform_chain():
    sg = pycvc_gl.SceneGraph(app)
    root = sg.add_group("root")
    root.setPosition(1000.0, 0.0, 0.0)
    child = sg.add_child_geometry("root", "child", _make_tri())
    child.setPosition(0.0, 20.0, 0.0)  # LOCAL, relative to root
    sg.processEvents()
    # child origin resolves through root∘child: (1000,0,0) + (0,20,0).
    w = list(child.local_to_world([0.0, 0.0, 0.0]))
    assert math.isclose(w[0], 1000.0, abs_tol=1e-6)
    assert math.isclose(w[1], 20.0, abs_tol=1e-6)
    assert math.isclose(w[2], 0.0, abs_tol=1e-6)
    # a non-origin local point carries the same offset
    w2 = list(child.local_to_world([5.0, 0.0, 0.0]))
    assert math.isclose(w2[0], 1005.0, abs_tol=1e-6)
    # world_to_local is the exact inverse
    back = list(child.world_to_local([1000.0, 20.0, 0.0]))
    assert math.isclose(back[0], 0.0, abs_tol=1e-6)
    assert math.isclose(back[1], 0.0, abs_tol=1e-6)
    assert math.isclose(back[2], 0.0, abs_tol=1e-6)
    print("  ok: local_to_world/world_to_local compose the parent∘child chain")


def test_local_point_to_real_and_dimensions():
    sg = pycvc_gl.SceneGraph(app)
    root = sg.add_group("root")
    root.setPosition(1000.0, 0.0, 0.0)
    child = sg.add_child_geometry("root", "child", _make_tri())
    sg.processEvents()
    # 1 world unit == 1000 m: the child origin sits at world (1000,0,0) ->
    # 1,000,000 m -> the km tier, reported as (1000, 0, 0, "km").
    u = pycvc.world_units(1000.0)
    c = child.local_point_to_real([0.0, 0.0, 0.0], u)
    assert len(c) == 4
    assert c[3] == "km"
    assert math.isclose(c[0], 1000.0, rel_tol=1e-9)
    # real_dimensions: the node's own size in the regime (finite, non-negative).
    d = child.real_dimensions(u)
    assert len(d) == 4 and isinstance(d[3], str)
    assert all(math.isfinite(v) for v in d[:3])
    print("  ok: local_point_to_real / real_dimensions project into a regime")


def test_world_bounding_box_is_translated():
    sg = pycvc_gl.SceneGraph(app)
    node = sg.addGraphics("tri", _make_tri())
    node.setPosition(100.0, 200.0, 0.0)
    sg.processEvents()
    bb = list(node.get_world_bounding_box())
    assert len(bb) == 6 and all(math.isfinite(v) for v in bb)
    minx, miny, minz, maxx, maxy, maxz = bb
    assert minx <= maxx and miny <= maxy and minz <= maxz
    # the triangle's local origin corner moved to world x≈100, y≈200
    assert minx >= 99.0 and miny >= 199.0
    print("  ok: get_world_bounding_box reflects the world transform")


def test_pick_world_binding_present():
    # pickWorld needs a live SceneRenderer with a render window (a display), so
    # it cannot run in the headless CI harness. Assert the re-exposed binding is
    # present on the class (pickWorld's double[3] out-param -> pick_world tuple).
    assert hasattr(pycvc_gl.SceneRenderer, "pick_world")
    print("  ok: SceneRenderer.pick_world binding present (needs a display to run)")


# ── 2. Built-in reference grid + axis ────────────────────────────────────────


def test_grid_node_wrapped():
    sg = pycvc_gl.SceneGraph(app)
    grid = sg.getGridNode()
    assert grid is not None
    grid.set_bounds(-5.0, -5.0, -5.0, 5.0, 5.0, 5.0)
    gb = grid.get_bounds()
    assert len(gb) == 6
    assert math.isclose(gb[0], -5.0) and math.isclose(gb[3], 5.0)
    grid.setGridDivisions(4, 6, 8)
    assert grid.get_grid_divisions() == (4, 6, 8)
    grid.setTickIntervals(2, 3, 4)
    assert grid.get_tick_intervals() == (2, 3, 4)
    grid.setYZPlaneColor(0.1, 0.2, 0.3)
    c = grid.get_yz_plane_color()
    assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(c, (0.1, 0.2, 0.3)))
    grid.setTickLabelColor(0.4, 0.5, 0.6)
    assert all(
        math.isclose(a, b, abs_tol=1e-6)
        for a, b in zip(grid.get_tick_label_color(), (0.4, 0.5, 0.6))
    )
    grid.setTickLabelFontSize(14)
    assert grid.getTickLabelFontSize() == 14
    grid.setXYPlaneVisible(False)
    assert not grid.isXYPlaneVisible()
    grid.setXYPlaneVisible(True)
    assert grid.isXYPlaneVisible()
    print("  ok: getGridNode() -> bounds/divisions/tick colours/visibility round-trip")


def test_axis_node_wrapped():
    sg = pycvc_gl.SceneGraph(app)
    axis = sg.getAxisNode()
    assert axis is not None
    axis.setAxisLength(50.0)  # no getter; exercises the setter path
    axis.setVisible(False)
    assert not axis.isVisible()
    axis.setVisible(True)
    assert axis.isVisible()
    print("  ok: getAxisNode() -> setAxisLength/visibility")


# ── 3. Lights ────────────────────────────────────────────────────────────────


def test_light_node_wrapped():
    sg = pycvc_gl.SceneGraph(app)
    light = sg.addLight("key")
    assert light is not None
    light.set_kind("directional")
    assert light.kind_str() == "directional"
    light.set_kind("spot")
    assert light.kind_str() == "spot"
    try:
        light.set_kind("laser")
    except Exception:
        pass
    else:
        raise AssertionError("set_kind('laser') should have raised")
    light.setTarget(1.0, 2.0, 3.0)
    t = light.get_target()
    assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(t, (1.0, 2.0, 3.0)))
    light.setColor(0.25, 0.5, 0.75)
    col = light.get_color()
    assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(col, (0.25, 0.5, 0.75)))
    light.setIntensity(2.5)
    assert math.isclose(light.intensity(), 2.5, abs_tol=1e-6)
    light.setCone(30.0)
    assert math.isclose(light.cone(), 30.0, abs_tol=1e-6)
    wp = light.world_position()
    assert len(wp) == 3 and all(math.isfinite(v) for v in wp)
    # reachable again via the typed downcast and via getGraphics upgrade
    same = sg.light_node("key")
    assert same is not None and same.kind_str() == "spot"
    upgraded = sg.getGraphics("key")
    assert hasattr(upgraded, "kind_str"), "getGraphics upgrades a light to LightNode"
    print("  ok: addLight -> kind/target/colour/intensity/cone + light_node downcast")


# ── 4. VolRenNode + the volren raycaster value types ─────────────────────────


def test_volren_node_wrapped():
    sg = pycvc_gl.SceneGraph(app)
    vn = sg.add_volren("vr")
    assert vn is not None
    idx = vn.addVolume(_make_field(4))
    assert idx == 0
    assert vn.volumeCount() == 1
    bb = list(vn.get_bounding_box())
    assert len(bb) == 6 and all(math.isfinite(v) for v in bb)
    u = pycvc.world_units()
    dims = vn.volume_real_dimensions(0, u)
    assert len(dims) == 4 and isinstance(dims[3], str)
    assert all(math.isfinite(v) and v >= 0.0 for v in dims[:3])
    pt = vn.volume_point_to_real(0, 0.0, 0.0, 0.0, u)
    assert len(pt) == 4 and isinstance(pt[3], str)
    same = sg.volren_node("vr")
    assert same is not None and same.volumeCount() == 1
    print("  ok: add_volren -> addVolume/get_bounding_box/real-units + volren_node")


def test_volren_raycaster_value_types():
    if not hasattr(pycvc, "raycaster"):
        print("  skip: volren raycaster not built into pycvc")
        return
    rc = pycvc.raycaster(app)
    # camera eye/focal/up are std::array<double,3> — they cross as 3-tuples via
    # the pycvc_volren.i typemaps (the #1 marshalling hazard). Set + read back.
    cam = rc.view()
    cam.eye = (1.0, 2.0, 3.0)
    cam.focal = (0.0, 0.0, 0.0)
    cam.up = (0.0, 0.0, 1.0)
    assert tuple(cam.eye) == (1.0, 2.0, 3.0)
    assert tuple(rc.view().eye) == (1.0, 2.0, 3.0), "the camera aliases the raycaster"
    # render_settings.background is std::array<float,3> (float32 -> tolerance).
    rs = rc.settings()
    rs.background = (0.1, 0.2, 0.3)
    bg = tuple(rc.settings().background)
    assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(bg, (0.1, 0.2, 0.3)))
    # add a volume + read the (opaque bbox) scene bounds as a 6-tuple
    assert rc.add_volume(_make_field(3)) == 0
    assert rc.volume_count() == 1
    sb = rc.scene_bounds_bbox()
    assert len(sb) == 6 and all(math.isfinite(v) for v in sb)
    print("  ok: raycaster camera/background arrays cross as tuples; scene_bounds 6-tuple")


def test_volslice_node_wrapped():
    # The cvc::volslice view-aligned slice renderer as a scene node. Headless:
    # setVolume defers the GL texture upload to tick(), and config()/setConfig()
    # round-trip through the state tree (no GL) — so everything below runs without
    # a render context. tick()/planesRendered() DO upload textures, so we assert
    # their binding is present, not a live tick (mirrors SceneRenderer.pick_world).
    sg = pycvc_gl.SceneGraph(app)
    if not hasattr(sg, "add_volslice"):
        print("  skip: volslice node not built")
        return
    node = sg.add_volslice("slice")
    assert node is not None
    assert not node.hasVolume()
    node.setVolume(_make_field(8))
    assert node.hasVolume()

    # config() -> volslice_render_settings proxy; mutate its members + the nested
    # slice_params, then setConfig() writes them back through cvc::state.
    rs = node.config()
    rs.filter = pycvc.interpolation_nearest  # scoped enum -> flat int constant
    rs.opacity_correction = True
    rs.tf_auto_domain = False
    rs.window_min, rs.window_max = 0.0, 1.0
    rs.slices.quality = 0.75
    rs.slices.max_planes = 500
    rs.slices.near_plane = 0.1
    assert rs.tf.point_count() == 0  # the shared volren TF crosses in (empty default)
    node.setConfig(rs)
    got = node.config()
    assert got.slices.max_planes == 500
    assert abs(got.slices.quality - 0.75) < 1e-9
    assert got.opacity_correction is True
    assert got.filter == pycvc.interpolation_nearest
    assert got.tf_auto_domain is False

    bb = node.get_bounding_box()  # opaque bbox -> finite 6-list (no GL)
    assert len(bb) == 6 and all(math.isfinite(v) for v in bb)

    # tick()/planesRendered() need a live GL context — presence only.
    assert callable(node.tick)
    assert callable(node.planesRendered)

    # typed downcast surfaces the concrete slice API regardless of getGraphics.
    same = sg.volslice_node("slice")
    assert same is not None and same.hasVolume()
    print("  ok: add_volslice -> setVolume/config round-trip/get_bounding_box + volslice_node")


# ── 5. On-screen overlays (state_object controllers, not scene nodes) ─────────


def test_stage_lighting_wrapped():
    # StageLighting(sg) is headless: the ctor + setStage/apply build LightNodes
    # into the scene graph (no GL context); they render wherever the scene does.
    sg = pycvc_gl.SceneGraph(app)
    rig = pycvc_gl.StageLighting(sg)
    rig.setStage(1.0, 2.0, 3.0, 10.0)
    assert all(math.isclose(a, b) for a, b in zip(rig.get_stage(), (1.0, 2.0, 3.0, 10.0)))
    # explicit intensities so lights are definitely built, then apply()
    rig.setKey(1.0, 45.0, 30.0, 35.0)
    rig.setFill(0.4)
    rig.setWash(0.5, 4, 1.0)
    rig.apply_preset("dramatic")  # string route over the Preset enum
    rig.apply()
    names = list(rig.lightNames())
    assert len(names) > 0 and any(n.startswith("stage_") for n in names)
    try:
        rig.apply_preset("disco")
    except Exception:
        pass
    else:
        raise AssertionError("apply_preset('disco') should have raised")
    print("  ok: StageLighting(sg) -> stage/preset/apply builds named stage_* lights")


def test_screen_text_hud_wrapped():
    # Headless via the low-level (app, statePath, None) ctor — no viewer, no
    # vtkTextActor; the change-gated setters write through state and read back.
    # (The live SceneRenderer& ctor + the actual draw need a GL context.)
    hud = pycvc_gl.ScreenTextHud(app, "scene.viewers.main.hud.caption", None)
    hud.setText("hello world")
    assert hud.text() == "hello world"
    hud.setPosition(0.5, 0.1)
    assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(hud.get_position(), (0.5, 0.1)))
    hud.setColor(1.0, 0.8, 0.6)
    assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(hud.get_color(), (1.0, 0.8, 0.6)))
    hud.setFontSize(24)
    assert hud.fontSize() == 24
    hud.setCentered(False)
    assert hud.centered() is False
    hud.setOpacity(0.5)
    assert math.isclose(hud.opacity(), 0.5, abs_tol=1e-6)
    print("  ok: ScreenTextHud (headless ctor) -> text/position/color/font round-trip")


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("pycvc_gl world tests: OK")
