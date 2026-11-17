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


def test_the_papers_automatic_mode():
    """seed_mode::critical_points (P6): critical_point_seeds, its job, and segment_seeded with no
    point classes."""
    vol = blobs()
    seeds = pycvc.critical_point_seeds(vol)
    assert list(seeds.class_names()) == ["feature", "background"], seeds.class_names()
    feature = seeds.class_index("feature")
    assert feature == 0
    # the two touching blobs make one peak, in the middle
    assert [12, 12, 12] in [list(p) for p in seeds.points("feature")]
    # Keywords, an enum by name (hyphens read as underscores), and the params class.
    moved = pycvc.critical_point_seeds(vol, cp_feature_threshold=100.0)
    assert len(moved.points("feature")) == 0
    p = pycvc.seeded_segmentation_params(cp_iterations=4, cp_sample=20)
    p.set_text("cp_field", "gradient_magnitude")
    p.set_text("cp_weights", "initial")
    assert p.validate() == [] and p.get_text("cp_field") == "gradient_magnitude"
    pycvc.critical_point_seeds(vol, p)
    try:
        pycvc.critical_point_seeds(vol, cp_k=-1.0)
        raise AssertionError("no ValueError")
    except ValueError as e:
        assert "cp_k" in str(e), e
    # The job.
    job = pycvc.critical_point_seeds_job(vol)
    while not job.step(256):
        pass
    assert list(job.result().class_names()) == ["feature", "background"]
    # The fully automatic segmentation: no point classes at all.
    r = pycvc.segment_seeded(vol, pycvc.point_classes(), automatic="critical-points")
    assert r.class_names == ["feature", "background"], r.class_names
    labels = r.labels_array()
    assert labels[12, 12, 9] == 0 and labels[0, 0, 0] == 1
    assert r.automatic_seeds == r.classified_seeds > 0
    print("  ok: the paper's mode,", len(seeds.points("feature")), "feature seeds")


def test_the_seed_generators():
    """VolUtils' seed generators (transfix/libcvc#123): capsid_critical_points, rotated_seeds."""
    vol = blobs()
    c = pycvc.capsid_critical_points(vol, t_low=30, h=1, k=1)
    assert list(c.class_names()) == ["maxima", "saddles"]
    assert [12, 12, 12] in c.points("maxima")
    p = pycvc.capsid_seed_params(h=3, k=0)
    assert p.validate() == []
    pycvc.capsid_critical_points(vol, p)
    try:
        pycvc.capsid_critical_points(vol, h=0, k=0)
        raise AssertionError("no ValueError")
    except ValueError as e:
        assert "h, k" in str(e), e
    one = pycvc.point_classes()
    one.add("s", 12, 12, 12)
    four = pycvc.rotated_seeds(one, vol, folds=4)
    assert list(four.class_names()) == ["Class 0", "Class 1", "Class 2", "Class 3"]
    assert four.points("Class 1") == [[11, 12, 12]], four.points("Class 1")
    two = pycvc.rotated_seeds(one, vol, folds=2, axis=[1.0, 0.0, 0.0], center=[0.0, 0.0, 0.0])
    assert two.points("Class 1") == [], "a half turn about x through the origin leaves the volume"
    print("  ok: the seed generators,", len(c.points("maxima")), "maxima")


if __name__ == "__main__":
    test_keywords_and_the_result()
    test_bad_input_raises()
    test_other_threads_run_meanwhile()
    test_the_job_steps_and_cancels()
    test_the_papers_automatic_mode()
    test_the_seed_generators()
    print("test_pycvc_segmentation: all passed")
