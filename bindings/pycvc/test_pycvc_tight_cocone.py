"""pycvc.tight_cocone, tight_cocone_params and tight_cocone_job (pycvc_tight_cocone.i, pycvc_jobs.i):
the VolumeRover TightCocone port's T11 rows (docs/roadmap/VOLROVER-TEXMOL-APPS-ROADMAP.md §2.4, V1;
VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7): keywords, the result's parts, a bad input's ValueError,
another Python thread running during the call (the GIL released), and a job's steps and cancel."""

import math
import threading
import time

import pycvc

app = pycvc.make_app()


def sphere(n=600, r=1.0):
    """n points on a sphere (a Fibonacci spiral, a little off its lattice), as [x, y, z, ...]."""
    golden = math.pi * (3.0 - math.sqrt(5.0))
    xyz = []
    for i in range(n):
        z = 1.0 - 2.0 * (i + 0.5) / n
        s = math.sqrt(1.0 - z * z)
        a = golden * i + 0.1 * math.sin(7.0 * i)
        xyz += [r * s * math.cos(a), r * s * math.sin(a), r * z]
    return xyz


def geometry(xyz):
    g = pycvc.geometry(app)
    g.add_vertices(xyz)
    return g


def closed(surface):
    """Whether every edge of the triangles is used twice, once each way."""
    edges = {}
    tris = surface.triangle_indices()
    for t in range(surface.num_triangles()):
        a, b, c = int(tris[t, 0]), int(tris[t, 1]), int(tris[t, 2])
        for u, v in ((a, b), (b, c), (c, a)):
            edges[(u, v)] = edges.get((u, v), 0) + 1
    return all(n == 1 and edges.get((v, u)) == 1 for (u, v), n in edges.items())


def test_a_sphere_gives_a_closed_surface_through_every_point():
    r = pycvc.tight_cocone(geometry(sphere()))
    s = r.surface
    assert s.num_triangles() == 2 * 600 - 4, r  # a closed genus-0 surface over 600 points
    assert len(r.point_index) == 600 and r.points == 600
    assert sorted(r.point_index) == list(range(600))
    assert closed(s)
    assert r.inside_cells > 0 and r.inside_cells <= r.cells
    assert list(r.warnings) == [], list(r.warnings)
    assert "triangles" in repr(r)


def test_keywords_and_params():
    p = pycvc.tight_cocone_params(robust=True, cocone_phi=0.3)
    assert p.robust and abs(p.cocone_phi - 0.3) < 1e-15
    d = p.to_dict()
    assert d["robust"] is True
    assert abs(pycvc.tight_cocone_params().big_ball_ratio - 1.0 / 16.0) < 1e-15
    try:
        pycvc.tight_cocone_params(no_such_field=1)
        raise AssertionError("an unknown keyword should raise")
    except TypeError as e:
        assert "cocone_phi" in str(e), e  # the valid keywords are listed
    r1 = pycvc.tight_cocone(geometry(sphere()), cocone_phi=0.3)
    r2 = pycvc.tight_cocone(geometry(sphere()), pycvc.tight_cocone_params(cocone_phi=0.3))
    assert r1.surface.num_triangles() == r2.surface.num_triangles()
    try:
        pycvc.tight_cocone(geometry(sphere()), p, robust=False)
        raise AssertionError("params and keywords together should raise")
    except TypeError:
        pass


def test_bad_input_raises_value_error():
    try:
        pycvc.tight_cocone(geometry([0, 0, 0, 1, 0, 0, 0, 1, 0]))
        raise AssertionError("three points should raise")
    except ValueError as e:
        assert "3 points" in str(e), e
    plane = []
    for i in range(30):
        plane += [i % 6, i // 6, 1.0]
    try:
        pycvc.tight_cocone(geometry(plane))
        raise AssertionError("points in one plane should raise")
    except ValueError as e:
        assert "one plane" in str(e), e
    try:
        pycvc.tight_cocone(geometry(sphere()), cocone_phi=5.0)
        raise AssertionError("a parameter out of range should raise")
    except (ValueError, TypeError) as e:
        assert "cocone_phi" in str(e), e


def test_other_threads_run_during_the_call():
    ticks = []
    stop = threading.Event()

    def ticker():
        while not stop.is_set():
            ticks.append(time.perf_counter())
            time.sleep(0.001)

    t = threading.Thread(target=ticker)
    t.start()
    try:
        before = len(ticks)
        pycvc.tight_cocone(geometry(sphere(8000)))
        during = len(ticks) - before
    finally:
        stop.set()
        t.join()
    assert during > 0, "the GIL was held through the call"


def test_the_job_steps_to_the_blocking_result_and_cancels():
    g = geometry(sphere())
    ref = pycvc.tight_cocone(g)
    job = pycvc.tight_cocone_job(g)
    phases, last = [], 0.0
    while not job.step(256):
        assert job.progress() >= last
        last = job.progress()
        if not phases or phases[-1] != job.phase():
            phases.append(job.phase())
    assert "tight cocone" in phases and "surface" in phases, phases
    r = job.result()
    assert r.surface.num_triangles() == ref.surface.num_triangles()
    assert list(r.point_index) == list(ref.point_index)
    job = pycvc.tight_cocone_job(g, robust=True)
    for _ in range(20):
        job.step(64)
    job.cancel()
    assert job.step(64) and job.cancelled() and not job.has_result()


def built_without_shewchuk():
    """libcvc without CVC_WITH_SHEWCHUK has no triangulation."""
    try:
        pycvc.tight_cocone(geometry(sphere(40)))
    except ValueError as e:
        return "without Shewchuk" in str(e)
    return False


if __name__ == "__main__":
    if built_without_shewchuk():
        print("skipped: libcvc built without Shewchuk's predicates (CVC_WITH_SHEWCHUK)")
        raise SystemExit(0)
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
