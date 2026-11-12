"""pycvc.skeletonize, skeleton_params and skeletonize_job (pycvc_skeleton.i, pycvc_jobs.i): the
VolumeRover Skeletonization port's T11 rows (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7,
§2): keywords, the result's parts, a bad input's ValueError, another Python thread running during
the call (the GIL released), and a job's steps and cancel."""

import math
import threading
import time

import pycvc

app = pycvc.make_app()


def capsule(length=8.0, r=1.0, nring=20, nalong=40):
    """A capsule along x, sampled on its surface, as a flat [x, y, z, ...] list."""
    xyz = []
    for i in range(nalong + 1):
        for k in range(nring):
            a = 2 * math.pi * (k + 0.37 * (i % 3)) / nring
            xyz += [length * i / nalong, r * math.cos(a), r * math.sin(a)]
    ncap = nring // 4
    for side in (0, 1):
        for i in range(1, ncap + 1):
            t = (math.pi / 2) * i / (ncap + 0.5)
            nr = max(1, int(round(nring * math.cos(t))))
            for k in range(nr):
                a = 2 * math.pi * (k + 0.21 * i) / nr
                x = -r * math.sin(t) if side == 0 else length + r * math.sin(t)
                xyz += [x, r * math.cos(t) * math.cos(a), r * math.cos(t) * math.sin(a)]
    return xyz


def geometry(xyz):
    g = pycvc.geometry(app)
    g.add_vertices(xyz)
    return g


def test_a_capsule_has_one_strand_along_its_axis():
    s = pycvc.skeletonize(geometry(capsule()))
    assert s.polylines == 1, s
    assert s.strands.num_lines() > 10
    assert s.sheets.num_triangles() == 0
    assert all(k == 0 for k in s.strand_of_line)
    assert "strands" in repr(s)


def test_keywords_and_params():
    p = pycvc.skeleton_params(robust=True, sheet_count=3, helix_window=True)
    assert p.robust and p.sheet_count == 3 and p.helix_window
    d = p.to_dict()
    assert d["sheet_count"] == 3
    try:
        pycvc.skeleton_params(no_such_field=1)
        raise AssertionError("an unknown keyword should raise")
    except TypeError as e:
        assert "sheet_count" in str(e), e  # the valid keywords are listed
    s1 = pycvc.skeletonize(geometry(capsule()), robust=True)
    s2 = pycvc.skeletonize(geometry(capsule()), pycvc.skeleton_params(robust=True))
    assert s1.polylines == s2.polylines == 1
    try:
        pycvc.skeletonize(geometry(capsule()), p, robust=False)
        raise AssertionError("params and keywords together should raise")
    except TypeError:
        pass


def test_bad_input_raises_value_error():
    try:
        pycvc.skeletonize(geometry([0, 0, 0, 1, 0, 0, 0, 1, 0]))
        raise AssertionError("three points should raise")
    except ValueError as e:
        assert "3 points" in str(e), e
    plane = []
    for i in range(30):
        plane += [i % 6, i // 6, 1.0]
    try:
        pycvc.skeletonize(geometry(plane))
        raise AssertionError("points in one plane should raise")
    except ValueError as e:
        assert "one plane" in str(e), e


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
        pycvc.skeletonize(geometry(capsule(nring=32, nalong=80)))
        during = len(ticks) - before
    finally:
        stop.set()
        t.join()
    assert during > 0, "the GIL was held through the call"


def test_the_job_steps_to_the_blocking_result_and_cancels():
    g = geometry(capsule())
    ref = pycvc.skeletonize(g)
    job = pycvc.skeletonize_job(g)
    phases, last = [], 0.0
    while not job.step(256):
        assert job.progress() >= last
        last = job.progress()
        if not phases or phases[-1] != job.phase():
            phases.append(job.phase())
    assert "tight cocone" in phases and "strands" in phases, phases
    s = job.result()
    assert s.polylines == ref.polylines and s.strands.num_lines() == ref.strands.num_lines()
    job = pycvc.skeletonize_job(g, sheet_count=1)
    for _ in range(20):
        job.step(64)
    job.cancel()
    assert job.step(64) and job.cancelled() and not job.has_result()


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
