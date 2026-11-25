"""pycvc's side of VolumeGridRover's Segmentation tab (pycvc_volume_grid_segment.i,
inc/cvc/volume/class_surfaces.h and volume_grid_segment.h): each class's surface of a label volume
with keywords and the params class, its job with a cancel, another Python thread running during
the call (the GIL released), bad input as ValueError; the tab's run as one job; the result's rows
(coloured as the seeds, on the state tree and back) and a slice's see-through layer as numpy. The
VolumeRover ports' T11 rows for P9 (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.4, §1.7)."""

import math
import threading
import time

import pycvc

app = pycvc.make_app()


def blobs(n=24):
    """Two Gaussian blobs along x (at n/4 and 3n/4) over a 0.02 background, as a float volume."""
    vals = []
    s2 = 2 * (n / 8.0) ** 2
    for k in range(n):
        for j in range(n):
            for i in range(n):
                r2 = (j - n / 2) ** 2 + (k - n / 2) ** 2
                vals.append(0.02 + math.exp(-((i - n / 4) ** 2 + r2) / s2)
                            + math.exp(-((i - 3 * n / 4) ** 2 + r2) / s2))
    vol = pycvc.volume(app)
    vol.set_float_grid(vals, n, n, n, 0, 0, 0, n - 1, n - 1, n - 1)
    return vol


def two_seeds(n=24):
    pc = pycvc.point_classes()
    pc.add("left", n // 4, n // 2, n // 2)
    pc.add("right", 3 * n // 4, n // 2, n // 2)
    return pc


def segmentation(n=24):
    return pycvc.segment_seeded(blobs(n), two_seeds(n), t_low=0.15, t_high=0.9, gamma=-0.04)


def test_class_surfaces_keywords_and_the_result():
    r = segmentation()
    s = pycvc.class_surfaces(r.labels, smoothing=4)
    assert s.labels == [0, 1], s.labels
    g = s.surface(0)
    assert g.num_triangles() > 0 and g.num_vertices() > 0
    assert set(s.surfaces) == {0, 1}
    assert s.voxels == {0: r.class_voxels[0], 1: r.class_voxels[1]}, s.voxels
    assert s.warnings == [] and s.triangle_count() > 0
    try:
        s.surface(7)
    except KeyError as e:
        assert "label 7" in str(e), str(e)
    else:
        raise AssertionError("no KeyError for a label with no surface")
    # The params class, its table and lists.
    p = pycvc.class_surface_params(smoothing=0, labels=[1])
    assert p.validate() == []
    only = pycvc.class_surfaces(r.labels, p)
    assert only.labels == [1]
    assert pycvc.class_surface_params().smoothing == 10
    print("  ok: class surfaces,", s)


def test_class_surfaces_bad_input_raises():
    vol = pycvc.volume(app)
    vol.set_float_grid([0.0] * 26 + [2.5], 3, 3, 3, 0, 0, 0, 2, 2, 2)
    try:
        pycvc.class_surfaces(vol)
    except ValueError as e:
        assert "not a label" in str(e), str(e)
    else:
        raise AssertionError("no ValueError for a voxel that is no label")
    r = segmentation(12)
    try:
        pycvc.class_surfaces(r.labels, mu=-0.1)
    except ValueError as e:
        assert "mu" in str(e), str(e)
    else:
        raise AssertionError("no ValueError for mu above -lambda")
    try:
        pycvc.class_surfaces(r.labels, smoothin=3)
    except TypeError as e:
        assert "smoothing" in str(e), str(e)
    else:
        raise AssertionError("no TypeError for an unknown keyword")
    print("  ok: bad input raises")


def test_other_threads_run_meanwhile():
    r = segmentation(64)
    ticks = [0]
    done = threading.Event()

    def tick():
        while not done.is_set():
            ticks[0] += 1
            time.sleep(0.001)

    t = threading.Thread(target=tick)
    t.start()
    t0 = time.time()
    pycvc.class_surfaces(r.labels, smoothing=20)
    elapsed = time.time() - t0
    done.set()
    t.join()
    assert ticks[0] > 0, "the other thread never ran during the call"
    print("  ok: %d ticks of another thread during %.2f s" % (ticks[0], elapsed))


def test_the_jobs_step_and_cancel():
    r = segmentation()
    ref = pycvc.class_surfaces(r.labels)
    job = pycvc.class_surfaces_job(r.labels)
    phases = []
    while not job.step(256):
        if not phases or phases[-1] != job.phase():
            phases.append(job.phase())
    assert "extracting" in phases and "smoothing" in phases, phases
    got = job.result()
    assert got.labels == ref.labels
    assert got.surface(0).num_triangles() == ref.surface(0).num_triangles()
    job2 = pycvc.class_surfaces_job(r.labels)
    job2.step(64)
    job2.cancel()
    assert job2.step(64) and job2.cancelled() and not job2.has_result()
    # The tab's run: the segmentation, then its surfaces, as one job.
    run = pycvc.volume_grid_segment_job(blobs(), two_seeds(), t_low=0.15, t_high=0.9,
                                        gamma=-0.04, mesh={"smoothing": 10})
    seen = []
    while not run.step(512):
        if not seen or seen[-1] != run.phase():
            seen.append(run.phase())
    assert "marching" in seen and "surfaces: extracting" in seen, seen
    res = run.result()
    assert res.segmentation.class_voxels == r.class_voxels
    assert (res.segmentation.labels_array() == r.labels_array()).all()
    assert res.surfaces.labels == [0, 1]
    assert res.surfaces.surface(1).num_triangles() == ref.surface(1).num_triangles()
    bare = pycvc.volume_grid_segment_job(blobs(), two_seeds(), t_low=0.15, t_high=0.9,
                                         gamma=-0.04, surfaces=False)
    while not bare.step(0):
        pass
    assert bare.result().surfaces.labels == []
    print("  ok: the jobs, phases", phases, seen)


def test_rows_and_the_colour_layer():
    r = segmentation()
    s = pycvc.class_surfaces(r.labels)
    seeds = two_seeds()
    rows = pycvc.segment_class_rows(seeds, 0, 0, r, s)
    assert [row.name for row in rows] == ["left", "right"]
    assert rows[0].color == tuple(seeds.classes[0].color), rows[0].color
    assert rows[0].visible and rows[0].voxels == r.class_voxels[0]
    assert rows[1].triangles == s.surface(1).num_triangles()
    # On the state tree and back: an edit's colour and shown flag.
    st = pycvc.State.instance(app).child("doc.grid.segment.rows")
    pycvc.write_segment_rows(st, rows)
    assert st.value() == "2" and st.child("1.name").value() == "right"
    st.child("0.color").value("0,1,0")
    st.child("1.visible").value("0")
    problems = []
    assert pycvc.read_segment_rows(st, rows, problems)
    assert rows[0].color == (0.0, 1.0, 0.0) and not rows[1].visible and problems == []
    # A slice of the labels as the see-through layer: the top row first.
    sl = pycvc.extract_slice(r.labels, pycvc.slice_axis_z, 12)
    px = pycvc.color_labels(sl, rows, 0.5)
    assert px.shape == (24, 24, 4) and px.dtype.name == "uint8"
    # row 12 from the top is the slice's row 11; the left blob's core (x 6) is class 0, green now
    assert list(px[24 - 1 - 12, 6]) == [0, 255, 0, 128], list(px[24 - 1 - 12, 6])
    assert px[24 - 1 - 12, 18, 3] == 0, "a hidden class is clear"
    assert px[0, 0, 3] == 0, "unlabelled is clear"
    # The tab's view parameters and which keys a session keeps.
    v = pycvc.volume_grid_segment_view_params(opacity=0.3)
    assert v.validate() == [] and v.show and v.live
    assert pycvc.volume_grid_segment_session_key("grid.segment.t_low")
    assert pycvc.volume_grid_segment_session_key("grid.segment.mesh.smoothing")
    assert not pycvc.volume_grid_segment_session_key("grid.segment.rows.0.color")
    print("  ok: rows and the colour layer")


if __name__ == "__main__":
    test_class_surfaces_keywords_and_the_result()
    test_class_surfaces_bad_input_raises()
    test_other_threads_run_meanwhile()
    test_the_jobs_step_and_cancel()
    test_rows_and_the_colour_layer()
    print("test_pycvc_volume_grid_segment: all passed")
