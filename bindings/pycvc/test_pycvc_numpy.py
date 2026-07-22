"""Robust tests for pycvc's zero-copy numpy views: data processing,
ownership/lifetime state, and threading.

The contract under test: a view returned by Geometry.vertices() /
Volume.grid() shares the C++ buffer (no copy), its `base` owns a shared_ptr
to the C++ storage (so the view stays valid after the facade object is
gone), and concurrent use is safe.
"""

import gc
import threading

import numpy as np
import pycvc


# ── Data processing: views share memory, both directions ────────────


def test_vertices_view_shares_memory():
    g = pycvc.geometry()
    g.add_vertices([0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0])
    v = g.vertices()
    assert v.shape == (4, 3) and v.dtype == np.float64
    assert v.base is not None  # the owning capsule
    # numpy -> C++
    v[2, 1] = 7.5
    assert g.vertices()[2, 1] == 7.5
    # C++ -> numpy (append rebinds the vector; existing view is a snapshot of
    # the old buffer, so re-fetch to see growth)
    g.add_vertex(5, 5, 5)
    assert g.vertices().shape == (5, 3)


def test_vertices_view_is_not_a_copy():
    g = pycvc.geometry()
    g.add_vertices([1.0, 2.0, 3.0])
    a = g.vertices()
    b = g.vertices()
    # two independent views of the SAME underlying buffer
    a[0, 0] = 99.0
    assert b[0, 0] == 99.0


def test_empty_geometry_view():
    g = pycvc.geometry()
    v = g.vertices()
    assert v.shape == (0, 3) and v.size == 0


def test_colors_view():
    g = pycvc.geometry()
    g.add_vertices([0, 0, 0, 1, 1, 1])
    g.set_colors([1, 0, 0, 0, 1, 0])
    c = g.vertex_colors()
    assert c.shape == (2, 3)
    assert np.allclose(c[0], [1, 0, 0]) and np.allclose(c[1], [0, 1, 0])


def test_volume_grid_view_and_indexing():
    nx, ny, nz = 2, 3, 4
    vals = [float(i) for i in range(nx * ny * nz)]
    vol = pycvc.volume()
    vol.set_float_grid(vals, nx, ny, nz, 0, 0, 0, 1, 1, 1)
    grid = vol.grid()
    assert grid.shape == (nz, ny, nx) and grid.dtype == np.float32
    # grid[k, j, i] == value(i, j, k)
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                assert grid[k, j, i] == vol.value(i, j, k)
    grid[1, 2, 0] = 123.0
    assert vol.value(0, 2, 1) == 123.0  # mutation reflected in C++


def test_grid_view_writability_flag():
    vol = pycvc.volume()
    vol.set_float_grid([1.0] * 8, 2, 2, 2, 0, 0, 0, 1, 1, 1)
    assert vol.grid().flags.writeable is True


# ── Ownership / lifetime state (the shared_ptr in the capsule base) ──


def test_view_outlives_facade_object():
    def make():
        g = pycvc.geometry()
        g.add_vertices([9, 9, 9, 8, 8, 8])
        return g.vertices()

    v = make()  # the Geometry is now unreferenced
    gc.collect()
    assert v.shape == (2, 3)
    assert float(v.sum()) == 51.0  # buffer still valid & correct


def test_volume_view_outlives_facade_object():
    def make():
        vol = pycvc.volume()
        vol.set_float_grid([float(i) for i in range(8)], 2, 2, 2, 0, 0, 0, 1, 1, 1)
        return vol.grid()

    g = make()
    gc.collect()
    assert g.shape == (2, 2, 2)
    assert float(g.sum()) == float(sum(range(8)))


def test_many_views_then_drop_source():
    g = pycvc.geometry()
    g.add_vertices(list(range(3 * 100)))
    views = [g.vertices() for _ in range(50)]
    del g
    gc.collect()
    # every view is still valid and consistent
    for v in views:
        assert v.shape == (100, 3)
    views[0][0, 0] = -1.0
    assert views[7][0, 0] == -1.0  # all share the one buffer


# ── Threading ───────────────────────────────────────────────────────


def test_threaded_build_and_view():
    errors = []

    def worker(seed):
        try:
            g = pycvc.geometry()
            g.add_vertices([float(seed)] * (3 * 64))
            v = g.vertices()
            assert v.shape == (64, 3)
            assert float(v[0, 0]) == float(seed)
            vol = pycvc.volume()
            vol.set_float_grid([float(seed)] * 64, 4, 4, 4, 0, 0, 0, 1, 1, 1)
            assert float(vol.grid()[0, 0, 0]) == float(seed)
        except Exception as e:  # noqa: BLE001
            errors.append(e)

    threads = [threading.Thread(target=worker, args=(i,)) for i in range(16)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert not errors, errors


def test_threaded_reads_of_shared_view():
    g = pycvc.geometry()
    g.add_vertices([float(i) for i in range(3 * 256)])
    view = g.vertices()
    results = []
    lock = threading.Lock()

    def reader():
        s = float(view.sum())
        with lock:
            results.append(s)

    threads = [threading.Thread(target=reader) for _ in range(16)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    expected = float(sum(range(3 * 256)))
    assert results and all(r == expected for r in results)


def test_writer_reader_thread_consistency():
    # One writer mutates the shared buffer; readers must never see torn/invalid
    # memory (GIL serializes element ops; the buffer stays alive throughout).
    vol = pycvc.volume()
    vol.set_float_grid([0.0] * (8 * 8 * 8), 8, 8, 8, 0, 0, 0, 1, 1, 1)
    grid = vol.grid()
    stop = threading.Event()

    def writer():
        i = 0
        while not stop.is_set():
            grid[0, 0, 0] = float(i % 100)
            i += 1

    def reader():
        for _ in range(1000):
            _ = float(grid[0, 0, 0])  # always a valid float, never a crash

    w = threading.Thread(target=writer)
    w.start()
    readers = [threading.Thread(target=reader) for _ in range(8)]
    for t in readers:
        t.start()
    for t in readers:
        t.join()
    stop.set()
    w.join()
    assert 0.0 <= float(grid[0, 0, 0]) < 100.0


# ── CUDA / unified-memory adapter ───────────────────────────────────


def test_gpu_adapter_host_build_degrades_cleanly():
    # libcvc stores GPU voxels in CUDA UNIFIED memory, so grid() (the host
    # numpy view) works whether data is on CPU or GPU. __cuda_array_interface__
    # additionally exposes the SAME unified buffer to cupy/torch on-device.
    # On a CUDA-disabled build (or host-resident data), on_gpu() is False and
    # there is no CUDA interface — the correct signal for GPU array libs.
    vol = pycvc.volume()
    vol.set_float_grid([1.0] * 8, 2, 2, 2, 0, 0, 0, 1, 1, 1)
    assert vol.on_gpu() is False
    assert vol.cuda_ptr() == 0
    try:
        _ = vol.__cuda_array_interface__
    except AttributeError:
        pass
    else:
        raise AssertionError("host-resident volume must not expose CAI")
    # the host numpy view is unaffected and would be identical for unified GPU
    assert np.asarray(vol.grid()).shape == (2, 2, 2)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("pycvc numpy/threading tests: OK")
