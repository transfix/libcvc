"""pycvc's point_classes (pycvc_point_classes.i, inc/cvc/volume/point_classes.h): marking points,
reading and writing .vgr and reading .pts through an app's handlers, numpy points, the parameter
tables' keywords and state round trip, errors, and another Python thread running while a file is
read. The VolumeRover ports' T11 row for P0d (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md
§1.7)."""

import os
import shutil
import tempfile
import threading
import time

import numpy as np

import pycvc

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "..", "src", "cvc", "tests", "data", "point_classes")


def sample():
    pc = pycvc.point_classes()
    pc.add("membrane", 12, 40, 7)
    pc.add("membrane", 13, 40, 7)
    pc.add("background", 0, 0, 0)
    pc.add("membrane", 1, 1, 1, 0, 2)
    return pc


def test_mark_and_inspect():
    pc = sample()
    assert len(pc.classes) == 3 and pc.point_count() == 4
    assert list(pc.class_names()) == ["membrane", "background", "membrane"]
    assert pc.points("membrane") == [[12, 40, 7], [13, 40, 7]]
    assert pc.points("membrane", 0, 2) == [[1, 1, 1]]
    a = pc.points_array("membrane")
    assert a.dtype == np.int32 and a.shape == (2, 3)
    assert a.tolist() == [[12, 40, 7], [13, 40, 7]]
    assert pc.add("membrane", 12, 40, 7) is False
    assert pc.remove("membrane", 12, 40, 7) is True
    assert tuple(pc.classes[0].points) == (13, 40, 7)
    assert tuple(pc.classes[0].color) == tuple(pycvc.point_class_palette(0))
    c = pc.classes[1]
    c.radius = 3.0
    c.color = [0, 0, 1]
    assert pc.classes[1].radius == 1.0, "a copy"
    pc.set_class(c)
    assert pc.classes[1].radius == 3.0 and tuple(pc.classes[1].color) == (0.0, 0.0, 1.0)
    assert pycvc.point_class_problems(pc) == ()
    assert pycvc.point_class_problems(pc, [13, 64, 64]) != ()
    try:
        pc.points("nothing")
    except ValueError as e:
        assert "no class 'nothing'" in str(e), str(e)
    else:
        raise AssertionError("an unknown class must raise ValueError")
    try:
        pc.add("membrane", -1, 0, 0)
    except ValueError as e:
        assert "not a voxel index" in str(e), str(e)
    else:
        raise AssertionError("a negative index must raise ValueError")
    print("  ok: add / remove / points / numpy / set_class / problems")


def test_files():
    app = pycvc.make_app()
    assert list(pycvc.point_class_read_extensions(app)) == [".pts", ".vgr"]
    assert list(pycvc.point_class_write_extensions(app)) == [".vgr"]
    d = tempfile.mkdtemp(prefix="pycvc_points_")
    try:
        pc = sample()
        path = os.path.join(d, "seeds.vgr")
        pycvc.write_point_classes(app, pc, path)
        assert pycvc.read_point_classes(app, path) == pc
    finally:
        shutil.rmtree(d, ignore_errors=True)
    legacy = pycvc.read_point_classes(app, os.path.join(DATA, "legacy.vgr"))
    assert list(legacy.class_names()) == ["Class 0", "membrane & lumen", "Class 0"]
    grid = pycvc.point_class_read_params(dims=[11, 11, 11], bbox=[0, 0, 0, 10, 10, 10])
    pts = pycvc.read_point_classes(app, os.path.join(DATA, "legacy.pts"), grid)
    assert pts.points("Class 0") == [[0, 0, 0], [10, 10, 10], [5, 2, 7]]
    print("  ok: .vgr round trip, the legacy .vgr and .pts")


def test_tables_keywords_and_state():
    app = pycvc.make_app()
    c = pycvc.point_class(name="seeds", color=[0, 1, 0], radius=2.5, timestep=3)
    node = pycvc.State.instance(app).child("doc.grid.classes.0")
    c.to_state(node)
    assert node.child("radius").value() == "2.5"
    back = pycvc.point_class.from_state(node)
    assert back.name == "seeds" and back.radius == 2.5 and back.timestep == 3
    assert pycvc.point_class(name="x", radius=0).validate() != []
    p = pycvc.point_class_read_params(dims=[1, 2])
    assert any("dims: 2 values" in m for m in p.validate()), p.validate()
    try:
        pycvc.point_class_read_params(dim=[1, 2, 3])
    except TypeError as e:
        assert "unknown keyword 'dim'" in str(e), str(e)
    else:
        raise AssertionError("an unknown keyword must raise TypeError")
    print("  ok: point_class / point_class_read_params keywords and state round trip")


def test_errors():
    app = pycvc.make_app()
    try:
        pycvc.read_point_classes(app, os.path.join(DATA, "nothing.vgr"))
    except RuntimeError as e:
        assert "nothing.vgr" in str(e), str(e)
    else:
        raise AssertionError("a missing file must raise RuntimeError")
    try:
        pycvc.read_point_classes(app, os.path.join(DATA, "legacy.vgr"),
                                 pycvc.point_class_read_params(dims=[1, 2, 3]))
    except ValueError as e:
        assert "dims and bbox go together" in str(e), str(e)
    else:
        raise AssertionError("bad parameters must raise ValueError")
    print("  ok: RuntimeError for files, ValueError for parameters")


def test_the_gil_is_released_while_reading():
    app = pycvc.make_app()
    big = pycvc.point_classes()
    for n in range(20000):
        big.add("c%d" % (n % 4), n % 97, n // 97, n % 13)
    d = tempfile.mkdtemp(prefix="pycvc_points_gil_")
    try:
        path = os.path.join(d, "big.vgr")
        pycvc.write_point_classes(app, big, path)
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
        for _ in range(5):
            back = pycvc.read_point_classes(app, path)
        took = time.perf_counter() - t0
        during = count[0] - before
        stop.set()
        t.join()
        assert back == big
        assert during > 0, "the other thread never ran during the read"
    finally:
        shutil.rmtree(d, ignore_errors=True)
    print("  ok: the GIL released: the other thread counted", during, "in five reads taking %.0f ms" %
          (1000 * took))


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("test_pycvc_point_classes: all passed")
