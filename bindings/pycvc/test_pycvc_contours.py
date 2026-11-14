"""pycvc's contour_stack (pycvc_contours.i, inc/cvc/geometry/contours.h): building a stack,
reading and writing every format through an app's handlers, the parameter tables' keywords and
state round trip, errors, and another Python thread running while a file is read. The VolumeRover
ports' T11 row for P0d (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7)."""

import gc
import math
import os
import shutil
import tempfile
import threading
import time

import pycvc

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "..", "src", "cvc", "tests", "data", "contours")


def sample():
    s = pycvc.contour_stack()
    s.z_spacing = 0.05
    s.add(2, "axon", [0, 0, 1, 0, 1, 1, 0, 1])
    s.add(3, "axon", [0.1, 0.2, 1.0 / 3.0, 0.2, 0.3, 1e-7])
    s.add(3, "dendrite", [5, 5, 6, 5, 5.5, 6.25])
    s.set_component(pycvc.contour_component(name="axon", color=[1, 0, 0]))
    s.set_component(pycvc.contour_component(name="dendrite", color=[0, 0.5, 0.25],
                                            visible=False))
    return s


def test_build_and_inspect():
    s = sample()
    assert s.indices() == [2, 3]
    assert list(s.component_names()) == ["axon", "dendrite"]
    assert s.polygon_count() == 3 and s.vertex_count() == 10
    assert s.polygons(3, "axon") == [[0.1, 0.2, 1.0 / 3.0, 0.2, 0.3, 1e-7]]
    assert s.polygons(9, "axon") == []
    assert math.isclose(s.slice_z(3), 0.15)
    s.set_slice_z(7, 2.5)
    assert s.has_slice(7) and s.slice_z(7) == 2.5
    comps = s.components
    assert comps[1].name == "dendrite" and comps[1].visible is False
    assert tuple(comps[0].color) == (1.0, 0.0, 0.0)
    # Copies: changing one changes nothing in the stack until set_component.
    c = s.components[0]
    c.color = [0, 0, 1]
    assert tuple(s.components[0].color) == (1.0, 0.0, 0.0)
    s.set_component(c)
    assert tuple(s.components[0].color) == (0.0, 0.0, 1.0)
    gc.collect()
    assert pycvc.read_contours  # the module still has its functions after a collection
    assert pycvc.contour_problems(s) == ()
    try:
        s.add(1, "axon", [0, 0, 1])
    except ValueError as e:
        assert "not x, y pairs" in str(e), str(e)
    else:
        raise AssertionError("an odd coordinate count must raise ValueError")
    print("  ok: add / components / polygons / slice planes")


def test_every_format_round_trips():
    app = pycvc.make_app()
    assert list(pycvc.contour_write_extensions(app)) == [".config", ".contour", ".cvcc", ".pcd",
                                                          ".ser"]
    d = tempfile.mkdtemp(prefix="pycvc_contours_")
    try:
        s = sample()
        for ext in (".cvcc", ".ser"):
            path = os.path.join(d, "s" + ext)
            pycvc.write_contours(app, s, path)
            back = pycvc.read_contours(app, path)
            assert back == s, ext
        pycvc.write_contours(app, s, os.path.join(d, "s.contour"))
        back = pycvc.read_contours(app, os.path.join(d, "s.contour"))
        assert list(back.component_names()) == ["material_1", "material_2"]
        assert back.polygons(3, "material_2") == s.polygons(3, "dendrite")
        one = pycvc.contour_stack()
        one.z_spacing = 0.5
        one.add(1, "stack", [0, 0, 1, 0, 0, 1])
        pycvc.write_contours(app, one, os.path.join(d, "stack.config"))
        assert pycvc.read_contours(app, os.path.join(d, "stack.config")) == one
        pycvc.write_contours(app, s, os.path.join(d, "s.pcd"))
        with open(os.path.join(d, "s.pcd")) as f:
            assert f.readline().strip() == "10"
    finally:
        shutil.rmtree(d, ignore_errors=True)
    print("  ok: .cvcc, .ser, .contour, .config round trips and .pcd")


def test_read_params_keywords_and_state():
    app = pycvc.make_app()
    ser = os.path.join(DATA, "reconstruct", "series.ser")
    s = pycvc.read_contours(app, ser)
    assert s.indices() == [1, 2, 3]
    assert list(s.component_names()) == ["d01", "spine & head"]
    assert s.polygons(2, "d01") == [[1, 1, 2, 1, 2, 2, 1, 2]]
    p = pycvc.contour_read_params(include=["d0[0-9]"], first=2)
    t = pycvc.read_contours(app, ser, p)
    assert t.indices() == [2, 3] and list(t.component_names()) == ["d01"]
    assert list(pycvc.contour_read_params.fields())[:3] == ["first", "last", "include"]
    node = pycvc.State.instance(app).child("doc.read")
    p.to_state(node)
    assert node.child("first").value() == "2"
    back = pycvc.contour_read_params.from_state(node)
    assert back.first == 2 and tuple(back.include) == ("d0[0-9]",)
    assert pycvc.contour_read_params(first=5, last=1).validate() == [
        "first = 5 is after last = 1"]
    try:
        pycvc.contour_read_params(frist=1)
    except TypeError as e:
        assert "unknown keyword 'frist'" in str(e), str(e)
    else:
        raise AssertionError("an unknown keyword must raise TypeError")
    c = pycvc.contour_component(name="a", color=[0.5, 0.5, 0.5])
    cn = pycvc.State.instance(app).child("doc.comp")
    c.to_state(cn)
    assert pycvc.contour_component.from_state(cn) == c
    print("  ok: contour_read_params / contour_component keywords and state round trip")


def test_errors():
    app = pycvc.make_app()
    try:
        pycvc.read_contours(app, os.path.join(DATA, "nothing.ser"))
    except RuntimeError as e:
        assert "nothing.ser" in str(e), str(e)
    else:
        raise AssertionError("a missing file must raise RuntimeError")
    try:
        pycvc.read_contours(app, os.path.join(DATA, "x.ser"), pycvc.contour_read_params(include=["("]))
    except ValueError as e:
        assert "not a regular expression" in str(e), str(e)
    else:
        raise AssertionError("a bad pattern must raise ValueError")
    bad = pycvc.contour_stack()
    bad.add(1, "a", [0, 0, 1, 1])
    try:
        pycvc.write_contours(app, bad, os.path.join(tempfile.gettempdir(), "never.cvcc"))
    except ValueError as e:
        assert "a polygon needs three" in str(e), str(e)
    else:
        raise AssertionError("a stack with problems must raise ValueError")
    print("  ok: RuntimeError for files, ValueError for parameters and problems")


def test_the_gil_is_released_while_reading():
    app = pycvc.make_app()
    big = pycvc.contour_stack()
    ring = []
    for k in range(400):
        a = 2 * math.pi * k / 400
        ring += [math.cos(a), math.sin(a)]
    for n in range(300):
        big.add(n, "tube", ring)
    d = tempfile.mkdtemp(prefix="pycvc_contours_gil_")
    try:
        path = os.path.join(d, "big.cvcc")
        pycvc.write_contours(app, big, path)
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
        back = pycvc.read_contours(app, path)
        took = time.perf_counter() - t0
        during = count[0] - before
        stop.set()
        t.join()
        assert back == big
        assert during > 0, "the other thread never ran during the read"
    finally:
        shutil.rmtree(d, ignore_errors=True)
    print("  ok: the GIL released: the other thread counted", during, "in a %.0f ms read" %
          (1000 * took))


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("test_pycvc_contours: all passed")
