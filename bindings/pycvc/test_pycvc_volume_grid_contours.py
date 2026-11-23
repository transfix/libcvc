"""VolumeGridRover's contours from Python (pycvc_volume_grid_contours.i, pycvc_jobs.i): the
VolumeRover ports' T11 rows for P10 (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7, §4): the
frame, an editor drawing and closing a curve, gestures, splines, the object rows on a State, stacks
both ways, JSON and the session, the isocontour tracer, the parameter tables' keywords, and the
tiler (keywords, the GIL released, a job's steps and cancel)."""

import math
import threading

import pycvc


def q8(v):
    """A coordinate on a 2^-8 grid, unchanged by the tiler's snapping."""
    return round(v * 256) / 256


def frame64():
    f = pycvc.grid_frame()
    f.dims = (64, 64, 64)
    return f


def tube(axis=pycvc.slice_axis_z, slices=5, n=16):
    c = pycvc.grid_contours()
    c.frame = frame64()
    o = pycvc.grid_contour_object(name="tube", color=[0.2, 0.8, 0.3])
    curves = []
    for k in range(slices):
        cv = pycvc.grid_curve()
        cv.axis = axis
        cv.index = 10 + k
        ru = []
        for i in range(n):
            t = 2 * math.pi * i / n
            w = 1 + 0.05 * math.cos(3 * t + 0.4 * k)
            ru += [q8(20 + 0.125 * k + 6 * w * math.cos(t)), q8(24 + 5 * w * math.sin(t))]
        cv.ru = ru
        cv.closed = True
        curves.append(cv)
    o.curves = curves
    c.objects = [o]
    return c


def test_the_frame_is_tuples():
    f = pycvc.grid_frame()
    f.dims = (10, 20, 30)
    f.origin = (1, 2, 3)
    f.spacing = (0.5, 0.25, 2)
    assert f.dims == (10, 20, 30)
    assert f.plane(pycvc.slice_axis_x, 4) == 3.0
    assert f.nearest_index(pycvc.slice_axis_z, 3 + 2 * 7.1) == 7
    try:
        f.dims = (1, 2)
        raise AssertionError("two dims taken")
    except ValueError:
        pass


def test_an_editor_draws_and_closes_a_curve():
    ed = pycvc.grid_contour_editor()
    ed.set_frame(frame64())
    assert ed.add_object(pycvc.grid_contour_object(name="axon")) == 0
    p = ed.add_point(0, pycvc.slice_axis_z, 12, 3.0, 4.0)
    assert (p.object, p.curve, p.point) == (0, 0, 0)
    ed.add_point(0, pycvc.slice_axis_z, 12, 9.0, 4.5)
    ed.add_point(0, pycvc.slice_axis_z, 12, 6.0, 9.0)
    assert ed.has_active() and ed.active_back()
    assert ed.close_active()
    c = ed.contours()
    assert c.objects[0].curves[0].closed
    assert list(c.objects[0].curves[0].ru) == [3, 4, 9, 4.5, 6, 9]
    assert ed.nearest_point(pycvc.slice_axis_z, 12, 9.1, 4.4, 0.5).point == 1
    assert ed.nearest_point(pycvc.slice_axis_z, 12, 30, 30, 0.5) is None
    assert ed.undo() and not ed.contours().objects[0].curves[0].closed
    try:
        ed.add_point(5, pycvc.slice_axis_z, 1, 0, 0)
        raise AssertionError("no such object")
    except IndexError:
        pass
    try:
        ed.add_object(pycvc.grid_contour_object(name="axon"))
        raise AssertionError("a name taken twice")
    except ValueError:
        pass


def test_gestures_turn_picks_into_edits():
    ed = pycvc.grid_contour_editor()
    ed.set_frame(frame64())
    ed.add_object(pycvc.grid_contour_object(name="axon"))
    g = pycvc.grid_contour_gestures()

    def pick(phase, r, u):
        p = pycvc.contour_pick()
        p.phase = phase
        p.axis = pycvc.slice_axis_z
        p.index = 7
        p.r, p.u = r, u
        p.inside = True
        return p

    for r, u in [(10, 10), (20, 10), (15, 20), (10, 10)]:
        g.apply(ed, 0, 1.0, False, pick(pycvc.contour_pick.phase_t_down, r, u))
        o = g.apply(ed, 0, 1.0, False, pick(pycvc.contour_pick.phase_t_up, r, u))
    assert "closed a curve of axon" in o.note, o.note
    g.apply(ed, 0, 1.0, False, pick(pycvc.contour_pick.phase_t_down, 0, 0))
    o = g.apply(ed, 0, 1.0, False, pick(pycvc.contour_pick.phase_t_drag, 30, 30))
    assert o.band and o.band_rect == (0, 0, 30, 30)


def test_splines_and_stacks():
    ru = [0, 0, 4, 0, 4, 4, 0, 4]
    s = pycvc.sample_grid_curve(ru, True, pycvc.contour_interpolation_spline, 5)
    assert len(s) == 40 and list(s[:2]) == [0, 0]
    assert list(pycvc.sample_grid_curve(ru, False, pycvc.contour_interpolation_linear, 5)) == ru
    c = tube()
    st = pycvc.grid_contours_stack(c)
    assert st.polygon_count() == 5 and list(st.indices()) == [10, 11, 12, 13, 14]
    assert pycvc.grid_contours_stack(c, pycvc.slice_axis_x).polygon_count() == 0
    back = pycvc.grid_contours_from_stack(st, c.frame)
    assert back == c
    bad = tube()
    o = bad.objects[0]  # a copy: the objects are set back whole
    o.samples = 0
    bad.objects = [o]
    assert pycvc.grid_contours_problems(bad)
    try:
        pycvc.grid_contours_stack(bad)
        raise AssertionError("contours with problems")
    except ValueError:
        pass


def test_rows_on_a_state():
    app = pycvc.make_app()
    rows = pycvc.State.instance(app).child("doc.grid.objects")
    c = tube()
    pycvc.write_contour_object_rows(rows, c)
    assert rows.value() == "1"
    assert rows.child("0.points").value() == "80"
    problems = []
    objs = pycvc.read_contour_object_rows(rows, problems)
    assert len(objs) == 1 and objs[0].name == "tube" and not problems
    rows.child("0.samples").value("0")
    pycvc.read_contour_object_rows(rows, problems)
    assert problems


def test_json_and_the_session_keep_everything():
    c = tube()
    o = c.objects[0]
    o.interpolation = pycvc.contour_interpolation_spline
    c.objects = [o]
    text = pycvc.grid_contours_json(c)
    assert '"interpolation": "spline"' in text
    assert pycvc.grid_contours_from_json(text) == c
    try:
        pycvc.grid_contours_from_json("{nope", "t.json")
        raise AssertionError("not JSON")
    except ValueError as e:
        assert "t.json" in str(e)
    s = pycvc.volume_grid_session(keys={"grid.tool": "contours"}, contours=c)
    back = pycvc.volume_grid_session_from_json(pycvc.volume_grid_session_json(s))
    assert back.contours == c


def test_the_tables_take_keywords():
    p = pycvc.volume_grid_contour_params(point_size=1.5, snap=True)
    assert p.point_size == 1.5 and p.snap
    try:
        pycvc.volume_grid_contour_params(nope=1)
        raise AssertionError("an unknown keyword")
    except TypeError:
        pass
    o = pycvc.grid_contour_object(name="m", samples=7)
    assert o.samples == 7 and not o.curves


def test_trace_a_slice():
    app = pycvc.make_app()
    vals = [math.hypot(i - 16, j - 15) - 8.0 for k in range(4) for j in range(32) for i in range(32)]
    vol = pycvc.volume(app)
    vol.set_float_grid(vals, 32, 32, 4, 0.0, 0.0, 0.0, 31.0, 31.0, 3.0)
    f = pycvc.grid_frame_of_volume(vol)
    assert f.dims == (32, 32, 4)
    loops = pycvc.trace_slice(pycvc.extract_slice(vol, pycvc.slice_axis_z, 2), f, 0.0)
    assert len(loops) == 1 and len(loops[0]) >= 6


def test_tile_with_keywords_the_gil_released_and_a_job():
    c = tube()
    r = pycvc.tile_grid_contours(c, curation_delta=0.001)
    assert list(r.names()) == ["tube"], r
    assert r.surface("tube").num_triangles() > 20
    assert r.surfaces["tube"].num_triangles() == r.surface("tube").num_triangles()
    # Another thread runs while the tiler does.
    ticks = []
    stop = threading.Event()

    def count():
        while not stop.is_set():
            ticks.append(1)

    t = threading.Thread(target=count)
    t.start()
    pycvc.tile_grid_contours(tube(slices=12, n=64))
    stop.set()
    t.join()
    assert ticks
    try:
        pycvc.tile_grid_contours(pycvc.grid_contours())
        raise AssertionError("nothing to tile")
    except ValueError:
        pass
    job = pycvc.tile_grid_contours_job(c, curation_delta=0.001)
    while not job.step(64):
        assert 0.0 <= job.progress() <= 1.0
    assert job.result().surface("tube").num_triangles() == r.surface("tube").num_triangles()
    stopped = pycvc.tile_grid_contours_job(tube(slices=12, n=64))
    stopped.step(16)
    stopped.cancel()
    assert stopped.step(16) and stopped.cancelled() and not stopped.has_result()


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
