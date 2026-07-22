"""Robust, deterministic tests for pycvc's threading model.

What the bindings actually guarantee (and what these tests therefore prove):

  * The C++ storage behind a view is reference-counted by a `shared_ptr` in the
    numpy array's capsule base, so views can be created/dropped from many
    threads without corrupting the refcount or freeing live memory.
  * Each facade owns its own buffer, so threads operating on their OWN
    Volume/Geometry are fully isolated (no cross-thread interference).
  * The CPython GIL serializes per-element numpy get/set, so a single-writer /
    many-reader pattern on a *shared* view never observes torn/garbage values.

What the bindings do NOT provide — and these tests deliberately do NOT assert —
is C++-level data-race freedom for *concurrent mutation* of one shared buffer.
The bindings add no locking; safe concurrency is a property of the access
*pattern* (per-object isolation, or single-writer under the GIL). Each test
below defines a safe pattern and proves it holds.

Coordination uses `threading.Barrier` / `threading.Event` (never sleeps) so the
threads genuinely overlap and the tests stay deterministic.
"""

import threading

import pycvc

N_THREADS = 8


def _run(target, n=N_THREADS, args_for=lambda i: (i,)):
    """Start n threads, join them, return the shared error list."""
    errors = []
    barrier = threading.Barrier(n)
    threads = [threading.Thread(target=target, args=(barrier, errors) + args_for(i)) for i in range(n)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return errors


# ── Per-thread isolation: each thread owns its data ─────────────────


def test_isolation_each_thread_owns_a_volume():
    def worker(barrier, errors, seed):
        try:
            barrier.wait()  # all threads hit the C++ app context together
            for _ in range(40):
                vol = pycvc.Volume()
                base = float(seed)
                vol.set_float_grid([base] * 64, 4, 4, 4, 0, 0, 0, 1, 1, 1)
                grid = vol.grid()
                grid += seed  # in-place on this thread's OWN buffer
                assert grid.shape == (4, 4, 4)
                assert float(grid[0, 0, 0]) == 2.0 * seed
                del vol  # view stays valid via the capsule
                assert float(grid[3, 3, 3]) == 2.0 * seed
                assert grid.max() == grid.min() == 2.0 * seed  # no bleed-through
        except Exception as e:  # noqa: BLE001
            errors.append(repr(e))

    errors = _run(worker)
    assert not errors, errors
    print("  ok: %d threads each own a Volume, fully isolated" % N_THREADS)


def test_isolation_each_thread_owns_a_geometry():
    def worker(barrier, errors, seed):
        try:
            barrier.wait()
            for _ in range(40):
                g = pycvc.Geometry()
                g.add_vertices([float(seed)] * (3 * 100))
                v = g.vertices()
                v += seed
                assert v.shape == (100, 3)
                del g
                assert float(v[0, 0]) == 2.0 * seed
                assert float(v[-1, -1]) == 2.0 * seed
        except Exception as e:  # noqa: BLE001
            errors.append(repr(e))

    errors = _run(worker)
    assert not errors, errors
    print("  ok: %d threads each own a Geometry, fully isolated" % N_THREADS)


# ── Concurrent readers of ONE shared view ───────────────────────────


def test_concurrent_readers_of_one_shared_view():
    # One immutable-during-read buffer, many concurrent readers. The shared
    # storage stays alive and every reader must compute the same, correct sum.
    nx, ny, nz = 8, 8, 8
    vol = pycvc.Volume()
    vals = [float(i) for i in range(nx * ny * nz)]
    vol.set_float_grid(vals, nx, ny, nz, 0, 0, 0, 1, 1, 1)
    view = vol.grid()
    expected = float(sum(vals))
    results = []
    lock = threading.Lock()

    def reader(barrier, errors):
        try:
            barrier.wait()
            for _ in range(200):
                s = float(view.sum())
                if s != expected:
                    raise AssertionError("reader saw %r, expected %r" % (s, expected))
            with lock:
                results.append(True)
        except Exception as e:  # noqa: BLE001
            errors.append(repr(e))

    errors = _run(reader, args_for=lambda i: ())
    assert not errors, errors
    assert len(results) == N_THREADS
    print("  ok: %d concurrent readers of a shared view all agree" % N_THREADS)


# ── Single writer / many readers on a shared view (GIL invariant) ───


def test_single_writer_many_readers_never_tear():
    # The bindings add no locking, but the GIL makes each numpy element get/set
    # atomic. With a SINGLE writer cycling a known value set, readers must only
    # ever observe values from that set (never torn/garbage), and the final
    # state is deterministic because we define the access pattern.
    vol = pycvc.Volume()
    vol.set_float_grid([0.0] * (8 * 8 * 8), 8, 8, 8, 0, 0, 0, 1, 1, 1)
    grid = vol.grid()
    valid = {float(x) for x in range(100)}
    stop = threading.Event()
    start = threading.Barrier(N_THREADS + 1)  # readers + writer
    errors = []

    def writer():
        try:
            start.wait()
            i = 0
            while not stop.is_set():
                grid[0, 0, 0] = float(i % 100)
                i += 1
        except Exception as e:  # noqa: BLE001
            errors.append(repr(e))

    def reader():
        try:
            start.wait()
            for _ in range(2000):
                x = float(grid[0, 0, 0])
                if x not in valid:
                    raise AssertionError("torn/invalid read: %r" % x)
        except Exception as e:  # noqa: BLE001
            errors.append(repr(e))

    w = threading.Thread(target=writer)
    readers = [threading.Thread(target=reader) for _ in range(N_THREADS)]
    w.start()
    for r in readers:
        r.start()
    for r in readers:
        r.join()
    stop.set()
    w.join()
    assert not errors, errors
    # Deterministic final state: quiesce the writer, then set a sentinel.
    grid[0, 0, 0] = 55.0
    assert vol.value(0, 0, 0) == 55.0
    print("  ok: single-writer/%d-reader shared view never tears; final state defined" % N_THREADS)


# ── Stress: build + view + drop churn to shake lifetime/GC races ─────


def test_stress_build_view_drop_churn():
    # Tight build/view/mutate/drop loop across threads: hammers the capsule
    # shared_ptr create/destroy path and the interpreter's GC concurrently.
    def worker(barrier, errors, seed):
        try:
            barrier.wait()
            acc = 0.0
            for it in range(150):
                if it % 2 == 0:
                    g = pycvc.Geometry()
                    g.add_vertices([float(seed + it)] * 12)  # 4 verts
                    view = g.vertices()
                    del g
                    view[0, 0] = float(seed)
                    acc += float(view[0, 0])
                    del view
                else:
                    vol = pycvc.Volume()
                    vol.set_float_grid([float(seed + it)] * 8, 2, 2, 2, 0, 0, 0, 1, 1, 1)
                    view = vol.grid()
                    del vol
                    view[0, 0, 0] = float(seed)
                    acc += float(view[0, 0, 0])
                    del view
            # 150 iterations, each adding `seed` once.
            assert acc == float(seed) * 150, (seed, acc)
        except Exception as e:  # noqa: BLE001
            errors.append(repr(e))

    errors = _run(worker)
    assert not errors, errors
    print("  ok: %d threads churn build/view/drop with no crash or corruption" % N_THREADS)


# ── CUDA under concurrency (guarded; runs iff cuda_available()) ──────


def test_cuda_isolation_under_concurrent_host_builds():
    if not pycvc.Volume.cuda_available():
        print("  skip: CUDA not available")
        return
    # One thread drives a GPU volume (enable -> coherence check -> disable)
    # while N-1 threads build & verify host volumes. Proves per-object
    # isolation: the GPU migration touches only its own buffer.
    n = N_THREADS
    barrier = threading.Barrier(n)
    errors = []

    def host_worker(seed):
        try:
            barrier.wait()
            for _ in range(30):
                vol = pycvc.Volume()
                vol.set_float_grid([float(seed)] * 64, 4, 4, 4, 0, 0, 0, 1, 1, 1)
                grid = vol.grid()
                assert vol.on_gpu() is False
                assert float(grid[0, 0, 0]) == float(seed)
                assert float(grid.max()) == float(seed)
        except Exception as e:  # noqa: BLE001
            errors.append(("host", repr(e)))

    def gpu_worker():
        try:
            barrier.wait()
            vol = pycvc.Volume()
            vol.set_float_grid([float(i) for i in range(64)], 4, 4, 4, 0, 0, 0, 1, 1, 1)
            vol.enable_cuda()
            try:
                assert vol.on_gpu() and vol.cuda_ptr() != 0
                grid = vol.grid()  # fresh view after migration
                assert grid.__array_interface__["data"][0] == vol.cuda_ptr()
                grid[2, 2, 2] = 314.0
                assert vol.value(2, 2, 2) == 314.0
                del grid
            finally:
                vol.disable_cuda()
            assert vol.value(2, 2, 2) == 314.0  # preserved back on host
        except Exception as e:  # noqa: BLE001
            errors.append(("gpu", repr(e)))

    threads = [threading.Thread(target=gpu_worker)]
    threads += [threading.Thread(target=host_worker, args=(s,)) for s in range(1, n)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert not errors, errors
    print("  ok: GPU enable/disable isolated from %d concurrent host builds" % (n - 1))


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
    print("pycvc threading-model tests: OK")
