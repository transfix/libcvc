"""pycvc.tile_contours, tile_params and tile_contours_job (pycvc_tiler.i, pycvc_jobs.i): the
VolumeRover ContourTiler port's T11 rows (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7, §3):
keywords, the result's surfaces and warnings, a bad input's ValueError, another Python thread
running during the call (the GIL released), and a job's steps and cancel."""

import math
import threading
import time

import pycvc


def q8(v):
    """A coordinate on a 2^-8 grid, unchanged by the tiler's snapping."""
    return round(v * 256) / 256


def ring(cx, cy, a, b, phase, n=24):
    """A wobbly ellipse, counterclockwise, as a flat [x0, y0, x1, y1, ...] list."""
    xy = []
    for i in range(n):
        t = 2 * math.pi * i / n
        r = 1 + 0.05 * math.cos(3 * t + phase)
        xy += [q8(cx + a * r * math.cos(t)), q8(cy + b * r * math.sin(t))]
    return xy


def tube(slices=5, n=24, two=False):
    s = pycvc.contour_stack()
    for k in range(slices):
        s.add(k, "tube", ring(8 + 0.125 * k, 8, 3 + 0.125 * (k % 2), 2.5, 0.4 * k, n))
        if two:
            s.add(k, "other", ring(24, 8, 2.5, 2.0, 0.3 * k, n))
    return s


def test_a_tube_is_one_closed_surface():
    r = pycvc.tile_contours(tube())
    assert list(r.names()) == ["tube"], r
    g = r.surface("tube")
    assert g.num_triangles() > 0
    assert list(r.surfaces) == ["tube"]
    assert r.surfaces["tube"].num_triangles() == g.num_triangles()
    assert r.warnings == []
    assert "1 surfaces" in repr(r)
    try:
        r.surface("nothing")
        raise AssertionError("a component with no surface should raise")
    except KeyError as e:
        assert "nothing" in str(e), e


def test_keywords_and_params():
    p = pycvc.tile_params(curation_delta=0.01, cap_ends=False, include=["tube"])
    assert p.curation_delta == 0.01 and not p.cap_ends and tuple(p.include) == ("tube",)
    assert p.to_dict()["cap_ends"] is False
    try:
        pycvc.tile_params(no_such_field=1)
        raise AssertionError("an unknown keyword should raise")
    except TypeError as e:
        assert "curation_delta" in str(e), e  # the valid keywords are listed
    s = tube(two=True)
    capped = pycvc.tile_contours(s)
    assert sorted(capped.names()) == ["other", "tube"]
    open_ = pycvc.tile_contours(s, p)
    assert list(open_.names()) == ["tube"]
    assert open_.surface("tube").num_triangles() < capped.surface("tube").num_triangles()
    same = pycvc.tile_contours(s, curation_delta=0.01, cap_ends=False, include=["tube"])
    assert same.surface("tube").num_triangles() == open_.surface("tube").num_triangles()
    try:
        pycvc.tile_contours(s, p, cap_ends=True)
        raise AssertionError("params and keywords together should raise")
    except TypeError:
        pass


def test_bad_input_raises_value_error():
    try:
        pycvc.tile_contours(tube(), curation_delta=-1)
        raise AssertionError("a negative curation_delta should raise")
    except ValueError as e:
        assert "curation_delta" in str(e), e
    try:
        pycvc.tile_contours(tube(), curation_delta=2)
        raise AssertionError("a curation_delta past the slice spacing should raise")
    except ValueError as e:
        assert "curation_delta" in str(e), e


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
        pycvc.tile_contours(tube(slices=40, n=120))
        during = len(ticks) - before
    finally:
        stop.set()
        t.join()
    assert during > 0, "the GIL was held through the call"


def test_the_job_steps_to_the_blocking_result_and_cancels():
    s = tube()
    ref = pycvc.tile_contours(s)
    job = pycvc.tile_contours_job(s)
    phases, last = [], 0.0
    while not job.step(16):
        assert job.progress() >= last
        last = job.progress()
        if not phases or phases[-1] != job.phase():
            phases.append(job.phase())
    assert "tiling" in phases and "assembling" in phases, phases
    r = job.result()
    assert r.surface("tube").num_triangles() == ref.surface("tube").num_triangles()
    job = pycvc.tile_contours_job(tube(slices=20, n=96), cap_ends=False)
    job.step(16)
    job.cancel()
    assert job.step(16) and job.cancelled() and not job.has_result()


def built_without_planar():
    """libcvc without cvc::planar (Clipper2, CDT, Shewchuk's predicates) has no tiler."""
    try:
        pycvc.tile_contours(tube(slices=2, n=8))
    except ValueError as e:
        return "without cvc::planar" in str(e)
    return False


if __name__ == "__main__":
    if built_without_planar():
        print("skipped: libcvc built without cvc::planar")
        raise SystemExit(0)
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
