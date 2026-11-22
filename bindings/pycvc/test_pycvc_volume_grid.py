"""pycvc's volume grid (pycvc_volume_grid.i, inc/cvc/volume/volume_grid.h), the data side of
VolumeGridRover: the point-class editor's edits, selection, moves, deletes and undo; the slice
views' gestures (a click toggles a point, a drag band selects, a Shift+Ctrl drag moves); the three
parameter tables' keywords and state round trip; the class rows on an app's state tree; sessions
through a file and through JSON; and the errors each raises. The VolumeRover ports' T11 row for P8
(docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7). These calls are cheap and keep the GIL, so
T11's other-thread-runs-meanwhile check has nothing to check here."""

import os
import shutil
import tempfile

import pycvc

Z, Y, X = pycvc.slice_axis_z, pycvc.slice_axis_y, pycvc.slice_axis_x
ADDED = pycvc.point_class_editor.toggled_added
REMOVED = pycvc.point_class_editor.toggled_removed
POINTS = pycvc.grid_tool_points


def raises(exc, fn, *args, contains=""):
    try:
        fn(*args)
    except exc as e:
        assert contains in str(e), "%r does not say %r" % (str(e), contains)
        return e
    raise AssertionError("%s%r must raise %s" % (fn.__name__, args, exc.__name__))


def two_classes():
    pc = pycvc.point_classes()
    pc.set_class(pycvc.point_class(name="membrane", radius=1.5))
    pc.set_class(pycvc.point_class(name="background", color=[0, 0, 1]))
    return pc


def editor(n=16):
    ed = pycvc.point_class_editor()
    ed.set_dims((n, n, n))
    ed.set_classes(two_classes(), False)
    return ed


def pick(phase, axis, i, j, k, mods=""):
    return pycvc.grid_pick(phase=phase, axis=axis, ijk=(i, j, k),
                           inside=all(0 <= c < 16 for c in (i, j, k)), mods=mods)


def test_views_axes_and_picks():
    assert pycvc.grid_view_axis(pycvc.grid_view_xy) == Z
    assert pycvc.grid_view_axis(pycvc.grid_view_xz) == Y
    assert pycvc.grid_view_axis(pycvc.grid_view_zy) == X
    for a in (X, Y, Z):
        assert pycvc.grid_view_axis(pycvc.grid_view_of(a)) == a
    assert pycvc.grid_plane_axes(Z) == (0, 1)
    assert pycvc.grid_plane_axes(Y) == (0, 2)
    assert pycvc.grid_plane_axes(X) == (2, 1)
    assert pycvc.grid_pick_phase_from("drag") == pycvc.grid_pick.phase_t_drag
    raises(ValueError, pycvc.grid_pick_phase_from, "hover", contains="'hover' is not a pick phase")
    p = pycvc.grid_pick()
    pycvc.parse_grid_pick_mods("ctrl,alt", p)
    assert (p.shift, p.ctrl, p.alt) == (False, True, True)
    p = pycvc.grid_pick(phase="up", axis=Y, ijk=[3, -2, 40], mods="shift")
    assert p.phase == pycvc.grid_pick.phase_t_up and p.axis == Y and p.shift and not p.ctrl
    assert p.ijk == (3, -2, 40) and p.inside is False, "a corner may lie past the volume"
    try:
        pycvc.grid_pick(phse="up")
    except TypeError as e:
        assert "unknown keyword 'phse'" in str(e) and "valid: phase" in str(e), str(e)
    else:
        raise AssertionError("an unknown keyword must raise TypeError")
    print("  ok: views, plane axes, pick phases, modifiers and keywords")


def test_editor_marks_points():
    ed = editor()
    assert ed.dims() == (16, 16, 16)
    assert ed.inside((15, 0, 15)) and not ed.inside([16, 0, 0])
    v0 = ed.version()
    assert ed.toggle(0, Z, (4, 5, 6)) == ADDED
    assert ed.version() > v0
    assert tuple(ed.classes().classes[0].points) == (4, 5, 6)
    # Radius 1.5: a click one voxel off on the plane finds it; `which` gets the point removed.
    which = pycvc.point_ref()
    assert ed.toggle(0, Z, (5, 5, 6), 1.0, which) == REMOVED
    assert which.cls == 0 and which.ijk == (4, 5, 6) and which == (0, (4, 5, 6))
    assert ed.classes().point_count() == 0
    assert ed.add(0, [4, 5, 6]) is True and ed.add(0, (4, 5, 6)) is False
    assert ed.toggle(0, Z, (4, 5, 7)) == ADDED, "another plane"
    assert ed.points_on(Z, 6) == [(0, (4, 5, 6))]
    assert ed.points_on(X, 4) == [(0, (4, 5, 6)), (0, (4, 5, 7))]
    # The pick radius scales with the display's point_scale.
    ed.add(1, (8, 8, 8))
    assert ed.nearest(Z, (10, 8, 8), 1.0) is None
    assert ed.nearest(Z, (10, 8, 8), 2.5) == (1, (8, 8, 8))
    assert ed.nearest(Z, (10, 8, 8), 2.5, 0) is None, "only class 0's"
    assert ed.pick_radius(1, 2.5) == 2.5 and ed.pick_radius(1, 0.1) == 0.5
    # A point is a (cls, (i, j, k)) pair or a point_ref.
    assert ed.remove((1, (8, 8, 8))) is True
    assert ed.remove(pycvc.point_ref(1, (8, 8, 8))) is False
    r = pycvc.point_ref(0, (4, 5, 6))
    assert r == (0, (4, 5, 6)) and r.as_tuple() == (0, (4, 5, 6))
    assert repr(r) == "point_ref(0, (4, 5, 6))"
    assert r < pycvc.point_ref(1, (0, 0, 0)) and len({r, pycvc.point_ref(0, (4, 5, 6))}) == 1
    r.ijk = [1, 2, 3]
    assert r.ijk == (1, 2, 3)
    # The shown variable and timestep: other classes are kept but neither shown nor edited.
    pc = ed.classes()
    c = pc.classes[1]
    c.timestep = 2
    c.points = [4, 4, 4]
    pc.set_class(c)  # a class of timestep 2: added beside the timestep-0 one
    ed.set_classes(pc)
    assert len(ed.classes().classes) == 3 and not ed.shown(2) and ed.shown(1)
    raises(ValueError, ed.toggle, 2, Z, (4, 4, 4))
    ed.set_shown(0, 2)
    assert (ed.shown_variable(), ed.shown_timestep()) == (0, 2) and ed.shown(2)
    assert ed.points_on(Z, 4) == [(2, (4, 4, 4))]
    print("  ok: toggle / add / remove / points_on / nearest / pick_radius / shown")


def test_editor_errors():
    ed = editor()
    raises(IndexError, ed.add, 5, (0, 0, 0))
    raises(ValueError, ed.add, 0, (16, 0, 0),
           contains="(16, 0, 0) lies outside the volume (16 x 16 x 16)")
    raises(ValueError, ed.set_dims, (4, 0, 4))
    raises(TypeError, ed.set_dims, (4, 4), contains="a sequence of 3 integers")
    raises(TypeError, ed.add, 0, (1.5, 0, 0))
    raises(TypeError, ed.select, "membrane", pycvc.selection_mode_add)
    assert ed.remove((9, (1, 1, 1))) is False
    print("  ok: IndexError for a class, ValueError for a voxel or dims, TypeError for shapes")


def test_selection_move_delete_undo():
    ed = editor()
    for i in range(5):
        ed.add(0, (i * 2, 3, 7))
    ed.add(1, (1, 9, 7))
    ed.add(1, (1, 9, 8))  # another plane
    sv = ed.selection_version()
    assert ed.select_rect(Z, 7, 0, 0, 4, 10, pycvc.selection_mode_replace) == 4
    assert ed.selection_version() > sv
    assert ed.selection() == [(0, (0, 3, 7)), (0, (2, 3, 7)), (0, (4, 3, 7)), (1, (1, 9, 7))]
    assert ed.selected((1, (1, 9, 7))) and not ed.selected((1, (1, 9, 8)))
    assert ed.select_rect(Z, 7, 8, 3, 6, 3, pycvc.selection_mode_add) == 6
    assert ed.select_rect(Z, 7, 0, 0, 2, 3, pycvc.selection_mode_remove) == 4
    ed.select((0, (0, 3, 7)), pycvc.selection_mode_replace)
    assert ed.selection() == [(0, (0, 3, 7))]
    ed.select_all()
    assert len(ed.selection()) == 7
    sel = ed.selection_classes()
    assert sel.point_count() == 7 and list(sel.class_names()) == ["membrane", "background"]
    ed.clear_selection()
    assert ed.selection() == []

    # Moves stay inside the volume and merge with the class's point they land on.
    ed = editor()
    ed.add(0, (1, 1, 4))
    ed.add(0, (14, 1, 4))
    ed.select((0, (1, 1, 4)), pycvc.selection_mode_add)
    assert ed.move_selection(Y, 2, 5) == (2, 5)  # down y: x right, z up
    assert ed.classes().classes[0].contains(3, 1, 9)
    assert ed.move_selection(Z, 40, 0) == (12, 0), "cut short at the edge"
    ed.move_selection(Y, -1, -5)
    assert ed.classes().classes[0].size() == 1 and ed.selection() == [(0, (14, 1, 4))]

    # Delete, undo, checkpoint.
    ed = editor()
    ed.add(0, (1, 2, 3))
    ed.add(0, (2, 2, 3))
    ed.add(1, (5, 5, 3))
    three = ed.classes()
    ed.select_rect(Z, 3, 0, 0, 15, 15, pycvc.selection_mode_replace)
    ed.select((0, (2, 2, 3)), pycvc.selection_mode_remove)
    assert ed.delete_selected() == 2 and ed.classes().point_count() == 1
    assert ed.delete_selected() == 0
    assert ed.undo() and ed.classes() == three
    depth = ed.undo_depth()
    ed.checkpoint()
    assert ed.undo_depth() == depth + 1
    while ed.undo():
        pass
    assert ed.classes().point_count() == 0, "set_classes(.., False) made no step"
    for i in range(100):
        ed.add(0, (i % 16, i // 16, 0))
    assert ed.undo_depth() == pycvc.point_class_editor.undo_limit == 64
    assert "100 points" in repr(ed), repr(ed)
    print("  ok: select / select_rect / selection_classes / move_selection / delete / undo")


def test_gestures():
    ed = editor()
    g = pycvc.point_class_gestures()
    # A click (down and up on one voxel) toggles a point of the current class.
    g.apply(ed, POINTS, 0, 1.0, pick("down", Z, 4, 5, 6))
    assert g.active()
    o = g.apply(ed, POINTS, 0, 1.0, pick("up", Z, 4, 5, 6))
    assert not g.active() and o.points_changed and o.note == "added (4, 5, 6) to membrane"
    assert tuple(ed.classes().classes[0].points) == (4, 5, 6)
    g.apply(ed, POINTS, 0, 1.0, pick("down", Z, 5, 5, 6))
    o = g.apply(ed, POINTS, 0, 1.0, pick("up", Z, 5, 5, 6))
    assert o.note == "removed (4, 5, 6) from membrane" and ed.classes().point_count() == 0
    g.apply(ed, POINTS, -1, 1.0, pick("down", Z, 5, 5, 6))
    o = g.apply(ed, POINTS, -1, 1.0, pick("up", Z, 5, 5, 6))
    assert not o.points_changed and "no class" in o.note

    # A drag is a rubber band that selects; Shift adds, Ctrl / Alt take away.
    ed = editor()
    for i in range(8):
        ed.add(i % 2, (i * 2, 6, 3))  # looking down y at 6: (r, u) = (x, z) = (2i, 3)

    def drag(r0, r1, mods):
        g.apply(ed, POINTS, 0, 1.0, pick("down", Y, r0, 6, 0, mods))
        o = g.apply(ed, POINTS, 0, 1.0, pick("drag", Y, r1, 6, 5, mods))
        assert o.band and o.band_axis == Y and o.band_index == 6
        assert o.band_rect == (r0, 0, r1, 5), o.band_rect
        return g.apply(ed, POINTS, 0, 1.0, pick("up", Y, r1, 6, 5, mods))

    o = drag(0, 6, "")
    assert o.note == "4 points selected" and o.selection_changed and not o.points_changed
    assert drag(10, 12, "shift").note == "6 points selected"
    assert drag(0, 2, "ctrl").note == "4 points selected"
    assert drag(4, 6, "alt").note == "2 points selected"
    assert ed.classes().point_count() == 8, "selecting edits nothing"
    d = o.to_dict()
    assert d["note"] == "4 points selected" and d["slice"] == (-1, -1, -1), d
    assert "selection_changed=True" in repr(o)

    # Shift+Ctrl drags the selection, one undo step for the whole drag.
    ed = editor()
    ed.add(0, (2, 2, 5))
    ed.add(1, (3, 2, 5))
    ed.select_all()
    depth = ed.undo_depth()
    g.apply(ed, POINTS, 0, 1.0, pick("down", Z, 2, 2, 5, "shift,ctrl"))
    for step in range(1, 5):
        o = g.apply(ed, POINTS, 0, 1.0, pick("drag", Z, 2 + step, 2 + step, 5, "shift,ctrl"))
        assert o.points_changed
    o = g.apply(ed, POINTS, 0, 1.0, pick("up", Z, 6, 6, 5, "shift,ctrl"))
    assert o.note == "moved 2 points"
    assert ed.classes().classes[0].contains(6, 6, 5) and ed.classes().classes[1].contains(7, 6, 5)
    assert ed.undo_depth() == depth + 1
    ed.undo()
    assert ed.classes().classes[0].contains(2, 2, 5)

    # Navigate moves the other two planes; probe edits nothing; reset drops a gesture.
    o = g.apply(ed, pycvc.grid_tool_navigate, 0, 1.0, pick("down", X, 9, 3, 12))
    assert o.slice == (-1, 3, 12)
    o = g.apply(ed, pycvc.grid_tool_navigate, 0, 1.0, pick("drag", X, 9, 30, -4))
    assert o.slice == (-1, 15, 0), "clamped to the volume"
    g.reset()
    assert not g.active()
    before = ed.classes()
    for ph in ("down", "drag", "up"):
        o = g.apply(ed, pycvc.grid_tool_probe, 0, 1.0, pick(ph, Z, 1, 1, 1))
    assert not o.points_changed and ed.classes() == before
    print("  ok: click toggles, band selects (Shift / Ctrl / Alt), Shift+Ctrl moves, navigate")


def test_params_tables():
    app = pycvc.make_app()
    grid = pycvc.State.instance(app).child("doc.graphics.root.children.grid.grid")
    p = pycvc.volume_grid_params(view=pycvc.grid_view_zy, tool=pycvc.grid_tool_navigate,
                                 slice_x=3, slice_z=40)
    assert p.validate() == []
    assert pycvc.volume_grid_params.fields() == ["view", "tool", "slice_x", "slice_y", "slice_z"]
    p.to_state(grid)
    assert grid.child("view").value() == "zy"
    assert grid.child("slice.x").value() == "3" and grid.child("slice.y").value() == "-1"
    assert grid.child("slice.z").property("unit") == "voxel"
    assert grid.child("tool").property("choices") == "navigate,points,probe,contours"
    back = pycvc.volume_grid_params.from_state(grid)
    assert back.view == pycvc.grid_view_zy and back.tool == pycvc.grid_tool_navigate
    assert back.slice_z == 40 and back.to_dict() == p.to_dict()

    display = grid.child("display")
    d = pycvc.volume_grid_display_params.from_state(display)  # seeds the defaults
    assert d.point_scale == 1.0 and d.show_points is True and d.grey is False
    assert display.child("point_scale").value() == "1"
    assert pycvc.volume_grid_display_params(point_scale=2.0, grey=True).validate() == []
    assert pycvc.volume_grid_display_params(point_scale=40.0).validate() != []
    display.child("point_scale").value("40")
    raises(ValueError, pycvc.volume_grid_display_params.from_state, display,
           contains="point_scale = 40 is above its maximum 10")

    s = pycvc.volume_grid_source_params.from_dict({"volume": "data.head.volume", "var": 1})
    assert s.to_dict() == {"volume": "data.head.volume", "points": "", "session": "", "var": 1,
                           "time": 0}
    source = grid.child("source")
    s.to_state(source)
    source.child("var").value("-1")
    raises(ValueError, pycvc.volume_grid_source_params.from_state, source,
           contains="var = -1 is below its minimum 0")
    try:
        pycvc.volume_grid_source_params(files="x")
    except TypeError as e:
        assert ("unknown keyword 'files'" in str(e) and
                "valid: volume, points, session, var, time" in str(e)), str(e)
    else:
        raise AssertionError("an unknown keyword must raise TypeError")
    print("  ok: the three tables' keywords, validate, from_dict and state round trip")


def test_class_rows_on_the_state_tree():
    app = pycvc.make_app()
    rows = pycvc.State.instance(app).child("doc.grid.classes")
    pc = two_classes()
    for p in ((1, 2, 3), (4, 5, 6)):
        pc.add("membrane", *p)
    pc.add("background", 7, 7, 7)
    pycvc.write_point_class_rows(rows, pc)
    assert rows.value() == "2"
    assert rows.child("0.name").value() == "membrane"
    assert rows.child("0.color").value() == "1,0,0"
    assert rows.child("0.radius").value() == "1.5"
    assert rows.child("0.points").value() == "2"
    problems = []
    back = pycvc.read_point_class_rows(rows, 0, 0, problems)
    assert problems == [] and len(back) == 2 and back[1].name == "background"
    assert pycvc.reconcile_point_class_rows(pc, back) == pc

    # A row edited in place keeps its points.
    rows.child("1.name").value("bg")
    rows.child("1.color").value("0.5,0.5,0.5")
    nxt = pycvc.reconcile_point_class_rows(pc, pycvc.read_point_class_rows(rows))
    assert nxt.classes[1].name == "bg" and tuple(nxt.classes[1].points) == (7, 7, 7)
    assert tuple(nxt.classes[1].color) == (0.5, 0.5, 0.5)

    # A list's Add: a row with only a name takes the shown variable and timestep.
    rows.child("2.name").value("class 3")
    rows.value("3")
    nxt = pycvc.reconcile_point_class_rows(nxt, pycvc.read_point_class_rows(rows, 1, 4))
    assert len(nxt.classes) == 3 and nxt.classes[2].points == ()
    assert (nxt.classes[2].variable, nxt.classes[2].timestep) == (1, 4)

    # Bad values are reported and defaulted.
    rows.child("0.radius").value("-2")
    rows.child("0.color").value("red")
    problems = []
    back = pycvc.read_point_class_rows(rows, 0, 0, problems)
    assert len(problems) == 2, problems
    assert any("classes.0.color: 'red' is not r,g,b" in m for m in problems), problems
    assert back[0].radius == 1.0

    # Two apps keep their own rows.
    other = pycvc.make_app()
    assert pycvc.State.instance(other).findDescendant("doc.grid.classes") is None
    print("  ok: rows written, read with problems and reconciled on an app's state tree")


def a_session():
    pc = two_classes()
    for p in ((1, 2, 3), (4, 5, 6)):
        pc.add("membrane", *p)
    return pycvc.volume_grid_session(
        keys={"grid.view": "xz", "grid.tool": "navigate", "grid.slice.z": "7",
              "grid.display.point_scale": "2",
              "grid.source.var": "1",
              "grid.classes.current": "1"},
        transfer_function='{"color":[[0,0,0,0],[1,1,1,1]]}',
        classes=pc)


def test_sessions():
    s = a_session()
    assert s.keys["grid.view"] == "xz" and s.classes.point_count() == 2
    s.classes.add("membrane", 9, 9, 9)
    assert s.classes.point_count() == 2, "session.classes is a copy"
    d = tempfile.mkdtemp(prefix="pycvc_grid_")
    try:
        path = os.path.join(d, "grid.session.json")
        pycvc.write_volume_grid_session(s, path)
        back = pycvc.read_volume_grid_session(path)
        assert back == s
        assert back.keys == s.keys and back.transfer_function == s.transfer_function
        assert back.classes == s.classes
        raises(RuntimeError, pycvc.read_volume_grid_session, os.path.join(d, "missing.json"),
               contains="missing.json")
        raises(RuntimeError, pycvc.write_volume_grid_session, s,
               os.path.join(d, "no", "such", "dir", "s.json"))
        bad = os.path.join(d, "key.json")
        with open(bad, "w") as f:
            f.write('{"format": "cvc.volume_grid.session", "keys": {"grid.busy": "1"}}')
        raises(RuntimeError, pycvc.read_volume_grid_session, bad,
               contains="'grid.busy' is not a key a session keeps")
    finally:
        shutil.rmtree(d, ignore_errors=True)
    text = pycvc.volume_grid_session_json(s)
    assert '"format": "cvc.volume_grid.session"' in text, text
    assert '"points": [1, 2, 3, 4, 5, 6]' in text, text
    assert pycvc.volume_grid_session_from_json(text) == s
    empty = pycvc.volume_grid_session_from_json(pycvc.volume_grid_session_json(
        pycvc.volume_grid_session()))
    assert empty.keys == {} and empty.classes.empty()
    raises(RuntimeError, pycvc.volume_grid_session_from_json, text[:40], "pasted",
           contains="pasted")
    assert pycvc.volume_grid_session_key("grid.slice.y")
    assert not pycvc.volume_grid_session_key("grid.busy")
    s.keys = dict(s.keys, **{"grid.request": "load"})
    raises(ValueError, pycvc.write_volume_grid_session, s, "unused.json",
           contains="'grid.request'")
    try:
        s.keys = {"grid.slice.z": 7}
    except TypeError as e:
        assert "str" in str(e), str(e)
    else:
        raise AssertionError("a key's value that is not text must raise TypeError")
    print("  ok: sessions through a file and through JSON; RuntimeError / ValueError / TypeError")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("test_pycvc_volume_grid: all passed")
