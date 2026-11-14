"""pycvc's seeded segmentation (pycvc_segmentation.i, inc/cvc/volume/segmentation.h), the
VolumeRover GenSeg port: keywords and the params class, numpy views of the labels, another
Python thread running during the call (the GIL released), bad input as ValueError, and the
resumable job with a cancel. The VolumeRover ports' T11 rows
(docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.4, §1.7)."""

import math
import threading
import time

import pycvc

app = pycvc.make_app()


def blobs(n=24):
    """Two touching Gaussian blobs along x over a 0.02 background, as a float volume."""
    vals = []
    for k in range(n):
        for j in range(n):
            for i in range(n):
                r2 = (j - n / 2) ** 2 + (k - n / 2) ** 2
                vals.append(0.02 + math.exp(-((i - 9) ** 2 + r2) / 24.5)
                            + math.exp(-((i - 15) ** 2 + r2) / 24.5))
    vol = pycvc.volume(app)
    vol.set_float_grid(vals, n, n, n, 0, 0, 0, n - 1, n - 1, n - 1)
    return vol


def two_seeds():
    pc = pycvc.point_classes()
    pc.add("left", 9, 12, 12)
    pc.add("right", 15, 12, 12)
    return pc


def test_keywords_and_the_result():
    vol = blobs()
    r = pycvc.segment_seeded(vol, two_seeds(), t_low=0.2, t_high=1.5)
    assert r.class_names == ["left", "right"]
    assert len(r.class_voxels) == 2 and min(r.class_voxels) > 0
    labels = r.labels_array()
    assert labels.shape == (24, 24, 24)
    # numpy's order is z, y, x
    assert labels[12, 12, 9] == 0 and labels[12, 12, 15] == 1 and labels[0, 0, 0] == 255
    assert sum(int((labels == c).sum()) for c in range(2)) == sum(r.class_voxels)
    assert r.labels.xdim() == 24 and labels.dtype.name == "uint8"
    assert len(r.classes) == 2
    c1 = r.classes[1].grid()
    assert c1[12, 12, 15] > 0.9 and c1[12, 12, 9] == 0.0
    border = r.border_array()
    assert border.shape[1] == 3 and border.shape[0] > 0
    assert r.automatic_seeds == 0 and r.warnings == []
    # The same through a params object, and an enum by name.
    p = pycvc.seeded_segmentation_params(t_low=0.2, t_high=1.5, class_volumes=False)
    assert p.validate() == []
    r2 = pycvc.segment_seeded(vol, two_seeds(), p)
    assert r2.classes == [] and r2.class_voxels == r.class_voxels
    r3 = pycvc.segment_seeded(vol, two_seeds(), t_low=0.2, t_high=1.5, automatic="none")
    assert r3.class_voxels == r.class_voxels
    print("  ok: keywords, the result, numpy views;", r)


def test_bad_input_raises():
    vol = blobs(12)
    for kw, what in [(dict(), "no seeds"), (dict(gamma=0.5), "gamma")]:
        seeds = pycvc.point_classes() if what == "no seeds" else two_seeds()
        try:
            pycvc.segment_seeded(vol, seeds, **kw)
        except ValueError as e:
            assert what in str(e), str(e)
        else:
            raise AssertionError("no ValueError for " + what)
    outside = pycvc.point_classes()
    outside.add("a", 20, 1, 1)
    try:
        pycvc.segment_seeded(vol, outside)
    except ValueError as e:
        assert "outside the 12x12x12 volume" in str(e), str(e)
    else:
        raise AssertionError("no ValueError for a seed outside the volume")
    try:
        pycvc.segment_seeded(vol, two_seeds(), t_lo=3)
    except TypeError as e:
        assert "t_low" in str(e), str(e)
    else:
        raise AssertionError("no TypeError for an unknown keyword")
    print("  ok: bad input raises")


def test_other_threads_run_meanwhile():
    vol = blobs(64)
    seeds = pycvc.point_classes()
    seeds.add("left", 9, 32, 32)
    seeds.add("right", 15, 32, 32)
    ticks = [0]
    done = threading.Event()

    def tick():
        while not done.is_set():
            ticks[0] += 1
            time.sleep(0.001)

    t = threading.Thread(target=tick)
    t.start()
    t0 = time.time()
    pycvc.segment_seeded(vol, seeds, t_low=0.0, t_high=10.0, gamma=-0.02)
    elapsed = time.time() - t0
    done.set()
    t.join()
    assert ticks[0] > 0, "the other thread never ran during the call"
    print("  ok: %d ticks of another thread during %.2f s" % (ticks[0], elapsed))


def test_the_job_steps_and_cancels():
    vol = blobs()
    ref = pycvc.segment_seeded(vol, two_seeds(), t_low=0.2, t_high=1.5)
    job = pycvc.segment_seeded_job(vol, two_seeds(), t_low=0.2, t_high=1.5)
    phases, last = [], 0.0
    while not job.step(256):
        assert job.progress() >= last
        last = job.progress()
        if not phases or phases[-1] != job.phase():
            phases.append(job.phase())
    # (checking two seeds can end inside the first step)
    assert phases[0] in ("checking", "normalising"), phases
    assert "marching" in phases and "writing" in phases, phases
    r = job.result()
    assert r.class_voxels == ref.class_voxels
    assert (r.labels_array() == ref.labels_array()).all()
    job2 = pycvc.segment_seeded_job(vol, two_seeds(), t_low=0.2, t_high=1.5)
    job2.step(64)
    job2.cancel()
    assert job2.step(64) and job2.cancelled() and not job2.has_result()
    print("  ok: the job, phases", phases)


if __name__ == "__main__":
    test_keywords_and_the_result()
    test_bad_input_raises()
    test_other_threads_run_meanwhile()
    test_the_job_steps_and_cancels()
    print("test_pycvc_segmentation: all passed")
