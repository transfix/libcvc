"""Ariadne data from Python (cvc/ariadne/data.h): datasets loaded into the state tree once, and read
back as the objects the scene draws.

pycvc.data_load(app, uri, ...) loads a file, a directory or an archive (here a gzip tar written
with Python's tarfile, as cvcpkg's data packages are) on a worker thread; data_wait waits for it;
data_files / data_file list the files and hand back their bytes; data_world / data_height_field
return a city bundle's world and terrain, sharing their arrays with the store. With a name, the
dataset is also linked at <prefix>.data.<name>, as a document's data: block does.
"""

import io
import json
import os
import shutil
import tarfile
import tempfile

import pycvc

TERRAIN = {
    "bounds": {"min_x": -10, "min_y": -10, "max_x": 10, "max_y": 10},
    "rows": 3,
    "cols": 3,
    "grid": [[2, 2, 2], [1, 1, 1], [0, 0, 0]],
}


def _package(path):
    """A city bundle as a cvcpkg data package holds it: a gzip tar, the bundle under share/."""
    with tarfile.open(path, "w:gz") as tar:
        for name, data in (
            ("share/libcvc-deps/scene-test/manifest.yaml", b"m"),
            ("share/cvc-scenes/test/terrain.json", json.dumps(TERRAIN).encode()),
            ("share/cvc-scenes/test/notes.txt", b"hello"),
        ):
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))


def test_a_directory_and_its_bytes():
    app = pycvc.make_app()
    d = tempfile.mkdtemp()
    try:
        os.makedirs(os.path.join(d, "set", "sub"))
        with open(os.path.join(d, "set", "a.txt"), "wb") as f:
            f.write(b"alpha")
        with open(os.path.join(d, "set", "sub", "b.bin"), "wb") as f:
            f.write(b"\x00\x01\xff")
        node = pycvc.data_load(app, os.path.join(d, "set"))
        assert node.startswith("data.set_"), node
        assert pycvc.data_wait(app, node, timeout=30), pycvc.state_get(app, node + ".error")
        assert pycvc.data_status(app, node) == "ready"
        assert list(pycvc.data_files(app, node)) == ["a.txt", "sub/b.bin"]
        assert pycvc.data_file(app, node, "sub/b.bin") == b"\x00\x01\xff"  # bytes, NUL and all
        assert pycvc.data_file(app, node, "missing") is None
    finally:
        shutil.rmtree(d)


def test_a_package_decodes_to_the_world_the_scene_draws():
    app = pycvc.make_app()
    d = tempfile.mkdtemp()
    try:
        pkg = os.path.join(d, "scene-test-1.0.0-any-noarch-none-none.tar.zst")  # cvcpkg's name
        _package(pkg)
        link = pycvc.data_load(
            app, pkg, path="share/cvc-scenes/test", decoder="city_bundle", name="city", prefix="doc"
        )
        assert pycvc.data_wait(app, "doc.data.city", timeout=30), pycvc.state_get(
            app, link + ".error"
        )
        assert link.startswith("data.scene_test_")  # the store node; doc.data.city links to it
        assert pycvc.data_status(app, "doc.data.city") == "ready"  # read through the link
        world = pycvc.data_world(app, "doc.data.city")
        terrain = pycvc.data_height_field(app, "doc.data.city")
        assert terrain.rows() == 3 and terrain.cols() == 3
        assert world.has_terrain() and list(world.names()) == ["terrain"]  # no buildings.glb
        assert float(pycvc.state_get(app, link + ".bounds.max_x")) == 10.0  # state_get: no links
        assert sorted(pycvc.data_files(app, "doc.data.city")) == ["notes.txt", "terrain.json"]
        # One file of the same package, its own request (as: text, from the extension).
        notes = pycvc.data_load(app, pkg, path="share/cvc-scenes/test/notes.txt")
        assert pycvc.data_wait(app, notes, timeout=30)
        assert pycvc.state_get(app, notes + ".text") == "hello"
    finally:
        shutil.rmtree(d)


def test_failures_say_why():
    app = pycvc.make_app()
    node = pycvc.data_load(app, "no/such/place.tar.gz")
    assert not pycvc.data_wait(app, node, timeout=30)
    assert "place.tar.gz" in pycvc.state_get(app, node + ".error")
    offline = pycvc.data_load(app, "https://example.org/x.tar.gz")
    assert not pycvc.data_wait(app, offline, timeout=30)
    assert "network is off" in pycvc.state_get(app, offline + ".error")
    try:
        pycvc.data_world(app, node)
        raise AssertionError("data_world of a failed dataset should raise")
    except RuntimeError as e:
        assert "holds no world_geometry" in str(e)
    try:
        pycvc.data_allow_network(app, ["https://cvcpkg.org/a/path"])
        raise AssertionError("an origin with a path should be refused")
    except RuntimeError as e:  # pycvc raises a C++ std::exception as RuntimeError
        assert "not an origin" in str(e)
    pycvc.data_allow_network(app, ["https://cvcpkg.org"])


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc data tests: OK")
