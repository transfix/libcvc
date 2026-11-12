"""pycvc's generic result_job wrapper (pycvc_jobs.i) on the contour tree and resize jobs
(cvc/volume/volume_jobs.h): step, done, cancel, progress, phase, status, result, with the GIL
released inside each step. The VolumeRover ports' T11 rows for the shared piece
(docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.4)."""

import gc
import random
import threading
import time

import pycvc

app = pycvc.make_app()


def noise_volume(n, seed):
    rng = random.Random(seed)
    vol = pycvc.volume(app)
    vol.set_float_grid([rng.random() for _ in range(n * n * n)], n, n, n, 0, 0, 0, 1, 1, 1)
    return vol


def test_steps_to_the_blocking_result():
    vol = noise_volume(24, 1)
    ref = pycvc.compute_contour_tree(vol)
    job = pycvc.contour_tree_job(vol)
    assert not job.done() and not job.has_result()
    last, phases, steps = 0.0, [], 0
    while not job.step(512):
        steps += 1
        assert job.progress() >= last
        last = job.progress()
        if not phases or phases[-1] != job.phase():
            phases.append(job.phase())
        assert job.status().endswith("%"), job.status()
    assert steps > 10
    assert phases[:2] == ["preparing", "sorting"], phases
    assert job.done() and job.has_result() and not job.failed() and not job.cancelled()
    assert job.phase() == "done" and job.status() == "done" and job.progress() == 1.0
    tree = job.result()
    assert len(tree.nodes) == len(ref.nodes) and len(tree.arcs) == len(ref.arcs)
    assert all(a.voxel == b.voxel and a.value == b.value for a, b in zip(tree.nodes, ref.nodes))
    assert list(tree.dims_list()) == [24, 24, 24]
    assert tree.components_at(0.5) == ref.components_at(0.5)
    assert job.steps() == steps + 1 and job.work_ms() >= job.max_step_ms() >= 0.0
    taken = job.take_result()
    assert len(taken.nodes) == len(ref.nodes)
    try:
        job.result()
    except RuntimeError as e:
        assert "already taken" in str(e)
    else:
        raise AssertionError("a taken result is gone")
    print("  ok: stepped to the blocking result,", steps + 1, "steps, phases", phases)


def test_options_and_run():
    vol = noise_volume(20, 2)
    opts = pycvc.contour_tree_options(max_dim=8)
    seen = []
    tree = pycvc.contour_tree_job(vol, opts).run(256, lambda ph, p: seen.append((ph, p)))
    assert list(tree.dims_list()) == [8, 8, 8]
    assert len(tree.nodes) == len(pycvc.compute_contour_tree(vol, opts).nodes)
    assert seen and all(0.0 <= p <= 1.0 for _, p in seen)
    print("  ok: run() with a progress callback,", len(seen), "calls")


def test_a_result_outlives_its_job():
    # result() is Python's own copy: the job may go, and its result may be taken, after it.
    tree = pycvc.contour_tree_job(noise_volume(12, 7)).run()  # the job is dropped at once
    gc.collect()
    assert list(tree.dims_list()) == [12, 12, 12] and len(tree.nodes) > 0
    job = pycvc.contour_tree_job(noise_volume(12, 8))
    job.step(0)
    kept = job.result()
    taken = job.take_result()
    del job
    gc.collect()
    assert len(kept.nodes) == len(taken.nodes) > 0
    # And a tree's nodes outlive the tree.
    nodes = pycvc.compute_contour_tree(noise_volume(12, 9)).nodes
    gc.collect()
    assert len(nodes) > 0 and all(n.voxel < 12 ** 3 for n in nodes)
    print("  ok: a result outlives its job and a take, and nodes outlive their tree")


def test_cancel_leaves_no_result():
    job = pycvc.contour_tree_job(noise_volume(20, 3))
    job.step(64)
    job.cancel()
    assert job.done() and job.cancelled()
    assert job.step(64)
    assert job.phase() == "cancelled" and job.status() == "cancelled"
    assert not job.has_result()
    try:
        job.result()
    except RuntimeError as e:
        assert "cancelled" in str(e)
    else:
        raise AssertionError("a cancelled job has no result")
    print("  ok: cancel")


def test_resize_job():
    vol = noise_volume(9, 4)
    job = pycvc.resize_job(vol, 5, 6, 7)
    assert job.phase() in ("starting", "running")
    out = job.run(16)
    assert (out.XDim(), out.YDim(), out.ZDim()) == (5, 6, 7)
    print("  ok: resize job (no phase of its own:", job.phase(), ")")


def test_the_gil_is_released_inside_a_step():
    # One long step (budget 0) on this thread while another Python thread counts: it counts only if
    # the step lets go of the GIL.
    job = pycvc.contour_tree_job(noise_volume(64, 5))
    count = [0]
    stop = threading.Event()

    def counter():
        while not stop.is_set():
            count[0] += 1

    t = threading.Thread(target=counter)
    t.start()
    time.sleep(0.01)
    before = count[0]
    t0 = time.perf_counter()
    job.step(0)
    took = time.perf_counter() - t0
    during = count[0] - before
    stop.set()
    t.join()
    assert job.has_result()
    assert during > 0, "the other thread never ran during the step"
    print("  ok: the GIL released: the other thread counted", during, "in a %.0f ms step" %
          (1000 * took))


def test_cancel_from_another_thread():
    job = pycvc.contour_tree_job(noise_volume(48, 6))
    timer = threading.Timer(0.001, job.cancel)
    timer.start()
    while not job.step(256):
        pass
    timer.join()
    # The cancel may land after the last step: either way the job is done, and only a job that was
    # not cancelled has its result.
    assert job.done()
    assert job.has_result() != job.cancelled()
    print("  ok: cancel from another thread (cancelled: %s)" % job.cancelled())


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("test_pycvc_jobs: all passed")
