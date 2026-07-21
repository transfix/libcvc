"""pycvc smoke test — build a mesh, round-trip it through file I/O.

Proves the SWIG bindings expose libcvc's geometry to Python end-to-end:
incremental + bulk builders, colors, normals, and .off save/load.
"""

import os
import tempfile

import pycvc


def test_incremental_build():
    g = pycvc.Geometry()
    a = g.add_vertex(0.0, 0.0, 0.0)
    b = g.add_vertex(1.0, 0.0, 0.0)
    c = g.add_vertex(0.0, 1.0, 0.0)
    assert (a, b, c) == (0, 1, 2)
    g.add_triangle(a, b, c)
    assert g.num_vertices() == 3
    assert g.num_triangles() == 1


def test_bulk_build_and_lines():
    g = pycvc.Geometry()
    g.add_vertices([0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0])  # 4 verts
    g.add_triangles([0, 1, 2, 0, 2, 3])  # 2 tris (a quad)
    g.add_lines([0, 1, 1, 2, 2, 3, 3, 0])  # 4 edges
    assert g.num_vertices() == 4
    assert g.num_triangles() == 2
    assert g.num_lines() == 4
    g.set_colors([1, 0, 0] * 4)  # per-vertex red
    g.compute_normals()


def test_bad_lengths_raise():
    g = pycvc.Geometry()
    for bad in (lambda: g.add_vertices([0, 0]), lambda: g.add_triangles([0, 1])):
        try:
            bad()
        except Exception:
            pass
        else:
            raise AssertionError("expected an exception on malformed bulk input")


def test_off_roundtrip():
    g = pycvc.Geometry()
    g.add_vertices([0, 0, 0, 1, 0, 0, 0, 1, 0])
    g.add_triangles([0, 1, 2])
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "tri.off")
        g.save(path)
        assert os.path.getsize(path) > 0
        h = pycvc.Geometry()
        h.load(path)
        assert h.num_vertices() == 3
        assert h.num_triangles() == 1


def test_volume_float_grid():
    nx, ny, nz = 4, 3, 2
    # ramp field: value == flat index, row-major (x fastest)
    vals = [float(i) for i in range(nx * ny * nz)]
    v = pycvc.Volume()
    v.set_float_grid(vals, nx, ny, nz, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0)
    assert (v.xdim(), v.ydim(), v.zdim()) == (nx, ny, nz)
    assert v.value(0, 0, 0) == 0.0
    assert v.value(nx - 1, ny - 1, nz - 1) == float(nx * ny * nz - 1)
    assert v.min_value() == 0.0
    assert v.max_value() == float(nx * ny * nz - 1)
    assert (v.xmin(), v.xmax()) == (-1.0, 1.0)


def test_volume_bad_length_raises():
    v = pycvc.Volume()
    try:
        v.set_float_grid([1.0, 2.0], 2, 2, 2, 0, 0, 0, 1, 1, 1)  # 2 != 8
    except Exception:
        pass
    else:
        raise AssertionError("expected an exception on wrong grid length")


def test_volume_rawiv_roundtrip():
    import os
    import tempfile

    nx, ny, nz = 4, 4, 4
    vals = [float(i % 7) for i in range(nx * ny * nz)]
    v = pycvc.Volume()
    v.set_float_grid(vals, nx, ny, nz, 0, 0, 0, 1, 1, 1)
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "field.rawiv")
        # .rawiv is the canonical CVC format and its handler is always
        # registered, so save() must succeed for a fresh path.
        v.save(path)
        assert os.path.getsize(path) > 0
        w = pycvc.Volume()
        w.load(path)
        assert (w.xdim(), w.ydim(), w.zdim()) == (nx, ny, nz)
        assert w.value(1, 0, 0) == v.value(1, 0, 0)


# ── Zero-copy numpy views ───────────────────────────────────────


def test_view_shares_memory_no_copy():
    import numpy as np

    g = pycvc.Geometry()
    g.add_vertices([1, 2, 3, 4, 5, 6])
    v = g.vertices()
    assert v.shape == (2, 3) and v.dtype == np.float64
    # The array is a VIEW: its base is our owner capsule, and it does not
    # own its data.
    assert v.base is not None and type(v.base).__name__ == "PyCapsule"
    assert v.flags.owndata is False


def test_view_mutation_is_bidirectional():
    g = pycvc.Geometry()
    g.add_vertices([0, 0, 0, 0, 0, 0])
    v = g.vertices()
    v[1, 2] = 7.5  # numpy write -> C++
    assert g.vertices()[1, 2] == 7.5  # fresh view sees it
    # two live views of the same object share storage
    a, b = g.vertices(), g.vertices()
    a[0, 0] = -3.0
    assert b[0, 0] == -3.0


def test_view_outlives_facade():
    import gc

    g = pycvc.Geometry()
    g.add_vertices([9, 8, 7, 6, 5, 4])
    v = g.vertices()
    del g
    gc.collect()  # facade gone; capsule's shared_ptr must keep data alive
    assert v[0, 0] == 9.0 and float(v.sum()) == 39.0


def test_empty_view_is_safe():
    g = pycvc.Geometry()
    v = g.vertices()  # no vertices
    assert v.shape == (0, 3)
    c = g.vertex_colors()  # no colors
    assert c.shape == (0, 3)


def test_volume_grid_view():
    import numpy as np

    nx, ny, nz = 5, 4, 3
    vals = [float(i) for i in range(nx * ny * nz)]
    vol = pycvc.Volume()
    vol.set_float_grid(vals, nx, ny, nz, 0, 0, 0, 1, 1, 1)
    g = vol.grid()
    assert g.shape == (nz, ny, nx) and g.dtype == np.float32
    assert g.base is not None and g.flags.owndata is False
    # index order matches operator()(i,j,k): grid[k,j,i]
    assert g[1, 2, 3] == vol.value(3, 2, 1)
    g[2, 3, 4] = 123.0
    assert vol.value(4, 3, 2) == 123.0


def test_threading_concurrent_build_and_view():
    # Data-processing + threading: many threads each build a mesh, take a
    # zero-copy view, mutate it, and verify — concurrently. Exercises the
    # shared_ptr refcounting / capsule lifetime under contention.
    import threading

    errors = []

    def worker(seed):
        try:
            for _ in range(50):
                g = pycvc.Geometry()
                g.add_vertices([float(seed)] * 300)  # 100 verts
                v = g.vertices()
                v += seed  # in-place on the C++ buffer
                assert v.shape == (100, 3)
                assert float(v[0, 0]) == 2.0 * seed
                del g  # view must stay valid via the capsule
                assert float(v[-1, -1]) == 2.0 * seed
        except Exception as e:  # noqa: BLE001
            errors.append(repr(e))

    threads = [threading.Thread(target=worker, args=(s,)) for s in range(1, 9)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert not errors, errors


def test_threading_shared_object_views():
    # Concurrent views of the SAME object from many threads (read + refcount
    # churn). Must not crash or corrupt; the shared C++ buffer is stable.
    import threading

    g = pycvc.Geometry()
    g.add_vertices([float(i) for i in range(3000)])  # 1000 verts
    errors = []

    def reader():
        try:
            for _ in range(200):
                v = g.vertices()
                assert v.shape == (1000, 3)
                _ = float(v.sum())
        except Exception as e:  # noqa: BLE001
            errors.append(repr(e))

    threads = [threading.Thread(target=reader) for _ in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert not errors, errors


def test_cuda_unified_memory():
    # Skips cleanly on CUDA-disabled builds / GPU-less machines. On a CUDA
    # build with a GPU it proves the single unified allocation serves numpy
    # (host) AND __cuda_array_interface__ (device) with no copies.
    if not pycvc.Volume.cuda_available():
        return
    nx, ny, nz = 4, 4, 4
    vol = pycvc.Volume()
    vol.set_float_grid([float(i) for i in range(nx * ny * nz)], nx, ny, nz, 0, 0, 0, 1, 1, 1)
    assert vol.on_gpu() is False  # host-resident by default
    vol.enable_cuda()
    assert vol.using_cuda() and vol.on_gpu()
    # host numpy view of the SAME unified buffer, zero-copy, writable
    g = vol.grid()
    g[0, 0, 0] = 77.0
    assert vol.value(0, 0, 0) == 77.0  # host write migrates + reflects
    # device interface for cupy/torch — same buffer, on-device zero-copy
    cai = vol.__cuda_array_interface__
    assert cai["shape"] == (nz, ny, nx)
    assert cai["typestr"] == "<f4"
    assert cai["data"][0] != 0  # valid device pointer
    vol.disable_cuda()
    assert vol.using_cuda() is False and vol.on_gpu() is False
    assert vol.value(0, 0, 0) == 77.0  # data preserved back on host


if __name__ == "__main__":
    test_incremental_build()
    test_bulk_build_and_lines()
    test_bad_lengths_raise()
    test_off_roundtrip()
    test_volume_float_grid()
    test_volume_bad_length_raises()
    test_volume_rawiv_roundtrip()
    test_view_shares_memory_no_copy()
    test_view_mutation_is_bidirectional()
    test_view_outlives_facade()
    test_empty_view_is_safe()
    test_volume_grid_view()
    test_threading_concurrent_build_and_view()
    test_threading_shared_object_views()
    test_cuda_unified_memory()
    print("pycvc smoke test: OK")
