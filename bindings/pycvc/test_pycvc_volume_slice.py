"""pycvc's axis-aligned slices (pycvc_volume_slice.i, inc/cvc/volume/volume_slice.h): slices down
each axis as numpy arrays, their colours through a transfer-function model, a file read one plane
at a time, the axis names, the parameter table's keywords and state round trip, errors, and another
Python thread running while planes are read. The VolumeRover ports' T11 row for P7
(docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7)."""

import os
import shutil
import tempfile
import threading

import numpy as np

import pycvc

app = pycvc.make_app()


def coded(nx=5, ny=4, nz=3):
    """voxel (i, j, k) = i + 100 j + 10000 k over [-2, 2] x [10, 13] x [0, 4]."""
    vals = [float(i + 100 * j + 10000 * k)
            for k in range(nz) for j in range(ny) for i in range(nx)]
    vol = pycvc.volume(app)
    vol.set_float_grid(vals, nx, ny, nz, -2.0, 10.0, 0.0, 2.0, 13.0, 4.0)
    return vol


def test_axes():
    assert pycvc.slice_axis_from("xy") == pycvc.slice_axis_z
    assert pycvc.slice_axis_from(" XZ ") == pycvc.slice_axis_y
    assert pycvc.slice_axis_from("zy") == pycvc.slice_axis_x
    assert pycvc.slice_axis_name(pycvc.slice_axis_y) == "y"
    assert pycvc.slice_view_name(pycvc.slice_axis_x) == "zy"
    assert pycvc.slice_plane_axes(pycvc.slice_axis_x) == (2, 1)
    try:
        pycvc.slice_axis_from("w")
        raise AssertionError("a bad axis must raise")
    except ValueError as e:
        assert "not an axis" in str(e)


def test_slices_down_each_axis():
    vol = coded()
    z = pycvc.extract_slice(vol, pycvc.slice_axis_z, 2)
    assert (z.width, z.height, z.index) == (5, 4, 2)
    a = z.values()
    assert a.shape == (4, 5) and a.dtype == np.float64
    assert a[3, 1] == 1 + 300 + 20000  # [row (y), column (x)]
    x = pycvc.extract_slice(vol, pycvc.slice_axis_x, 4).values()
    assert x.shape == (4, 3) and x[1, 2] == 4 + 100 + 20000  # z right, y up
    try:
        pycvc.extract_slice(vol, pycvc.slice_axis_z, 3)
        raise AssertionError("slice 3 of 3 must raise")
    except IndexError as e:
        assert "outside the 5x4x3 volume" in str(e)


def test_colours():
    vol = pycvc.volume(app)
    vol.set_float_grid([0.0, 5.0, 10.0, 10.0, float("nan"), 2.5], 3, 2, 1, 0, 0, 0, 2, 1, 0)
    s = pycvc.extract_slice(vol, pycvc.slice_axis_z, 0)
    gray = '{"color":[[0,0,0,0],[1,1,1,1]],"alpha":[[0,1],[1,1]]}'
    px = pycvc.color_slice(s, gray, 0.0, 10.0, True)
    assert px.shape == (2, 3, 4) and px.dtype == np.uint8
    assert px[0, 0].tolist() == [255, 255, 255, 255]  # the top row first: 10
    assert px[0, 1].tolist() == [0, 0, 0, 0]  # NaN
    assert px[1, 0].tolist() == [0, 0, 0, 255]  # 0
    assert abs(int(px[1, 1, 0]) - 128) <= 1  # 5
    try:
        pycvc.color_slice(s, "{not json", 0.0, 10.0, True)
        raise AssertionError("a bad model must raise")
    except ValueError:
        pass


def test_file_read_plane_by_plane():
    d = tempfile.mkdtemp(prefix="pycvc_slices_")
    try:
        vol = coded(6, 5, 4)
        path = os.path.join(d, "coded.rawiv")
        vol.save(path)
        mem = pycvc.slice_source(vol)
        disk = pycvc.slice_source(app, path, 0, 0, 3)
        assert mem.in_memory() and not disk.in_memory()
        assert disk.min() == 0.0 and disk.max() == 5 + 400 + 30000
        assert disk.count(pycvc.slice_axis_y) == 5 and disk.plane(pycvc.slice_axis_z, 2) == 8.0 / 3.0 * 2
        for axis in (pycvc.slice_axis_x, pycvc.slice_axis_y, pycvc.slice_axis_z):
            for s in range(mem.count(axis)):
                assert np.array_equal(disk.slice(axis, s).values(), mem.slice(axis, s).values())
        assert disk.reads() == 6 + 5 + 4
        assert disk.value(3, 2, 1, pycvc.slice_axis_y) == 3 + 200 + 10000
        # Another Python thread runs while planes are read.
        ticks = []
        stop = threading.Event()

        def count():
            while not stop.is_set():
                ticks.append(1)

        t = threading.Thread(target=count)
        t.start()
        for _ in range(20):
            for s in range(4):
                pycvc.slice_source(app, path).slice(pycvc.slice_axis_z, s)
        stop.set()
        t.join()
        assert ticks, "the other thread ran"
        try:
            pycvc.slice_source(app, os.path.join(d, "nope.rawiv"))
            raise AssertionError("a missing file must raise")
        except RuntimeError as e:
            assert "nope.rawiv" in str(e)
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_params_table():
    p = pycvc.volume_slice_params(axis=pycvc.slice_axis_x, index=7, opaque=False)
    assert p.validate() == []
    assert "overlay_offset" in pycvc.volume_slice_params.fields()
    node = pycvc.State.instance(app).child("doc.slice")
    p.to_state(node)
    back = pycvc.volume_slice_params.from_state(node)
    assert back.axis == pycvc.slice_axis_x and back.index == 7 and back.opaque is False
    p.index = -5
    assert p.validate()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
    print("OK")
