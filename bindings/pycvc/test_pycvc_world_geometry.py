"""pycvc: the world's one representation from Python (G4b of
docs/roadmap/WORLD-GEOMETRY-ROADMAP.md).

Python reads the same terrain as C++: ``pycvc.load_city_bundle`` is the C++ loader,
``pycvc.height_field.height`` the mesh's own triangles, and ``pymod_gl/scenes.py``'s
sampler is built on them instead of parsing ``terrain.json`` itself. The checks:
  1. a height field's grid, its numpy views and its write guard;
  2. ``height(x, y)`` is the triangle under the point (cells split south-west to
     north-east, as the physics engine splits them), not a bilinear blend;
  3. a world_geometry's items, placement, bounds and surface table;
  4. a bundle's north-first terrain.json comes back row 0 at min y, flipped once;
  5. scenes.py's sampler is height_field.height, clamped at the edges.
With ``CVC_CITY_BUNDLE`` pointing at a real bundle (e.g. austin_south), the Python
heights equal terrain.json's flipped grid sample for sample.
"""

from __future__ import annotations

import importlib.util
import json
import math
import os
import sys
import tempfile

import numpy as np

import pycvc

HERE = os.path.dirname(os.path.abspath(__file__))


def expect_raises(fn, what):
    try:
        fn()
    except RuntimeError:
        return
    raise AssertionError(what + ": no error raised")


def test_height_field_grid_and_views():
    h = np.arange(12, dtype=np.float64).reshape(3, 4)  # float64 in: narrowed
    f = pycvc.height_field(3, 4, 10.0, 20.0, 1.0, 2.0, h)
    assert isinstance(f, pycvc.geometry)
    assert (f.rows(), f.cols(), f.cell_rows(), f.cell_cols()) == (3, 4, 2, 3)
    assert f.bounds2d() == (10.0, 20.0, 13.0, 24.0)
    got = f.heights()
    assert got.shape == (3, 4) and got.dtype == np.float32
    assert np.array_equal(got, h.astype(np.float32))
    assert not got.flags.writeable
    for r in range(3):
        for c in range(4):
            assert f.height_at(r, c) == h[r, c]
            assert math.isclose(f.height(10.0 + c, 20.0 + 2.0 * r), h[r, c], abs_tol=1e-6)
    # The mesh: one vertex per sample, two triangles per cell, at the samples.
    m = f.mesh()
    assert m.num_vertices() == 12 and m.num_triangles() == 12
    v = m.vertices()
    assert np.allclose(v[f.vertex_of(2, 3)], (13.0, 24.0, 11.0))
    # The field's own mesh is read-only; the mesh() copy is editable and its own.
    assert not f.vertices().flags.writeable
    assert np.array_equal(f.vertices(), m.vertices())
    m.vertices()[0, 2] = 99.0
    assert f.vertices()[0, 2] == 0.0 and f.height_at(0, 0) == 0.0
    expect_raises(lambda: f.add_vertices([1.0, 2.0, 3.0]), "adding a vertex to the grid's mesh")
    expect_raises(lambda: f.load("terrain.off"), "reading a file over the grid's mesh")
    # Edits go through the grid.
    f.set_height(0, 0, 5.0)
    assert f.height_at(0, 0) == 5.0 and f.heights()[0, 0] == 5.0
    f.set_heights(np.zeros(12))
    assert float(np.abs(f.heights()).max()) == 0.0
    # clear() empties the grid with the mesh, never the mesh alone.
    f.clear()
    assert f.rows() == 0 and f.mesh().num_vertices() == 0
    expect_raises(lambda: pycvc.height_field(1, 4, 0, 0, 1, 1), "a 1-row grid")
    expect_raises(lambda: pycvc.height_field(3, 4, 0, 0, 1, 1, [1.0, 2.0]), "a short grid")


def test_height_is_the_triangle_not_bilinear():
    # One cell, only the north-east corner raised. Split south-west to north-east:
    # the centre lies on the diagonal (height 0.5; bilinear would say 0.25), and a
    # point in the south-east triangle (SW, SE, NE) rises with y alone.
    f = pycvc.height_field(2, 2, 0.0, 0.0, 1.0, 1.0, [0.0, 0.0, 0.0, 1.0])
    assert math.isclose(f.height(0.5, 0.5), 0.5, abs_tol=1e-6)
    assert math.isclose(f.height(0.75, 0.25), 0.25, abs_tol=1e-6)
    assert math.isclose(f.height(0.25, 0.75), 0.25, abs_tol=1e-6)  # NW triangle: z = x
    assert math.isnan(f.height(2.0, 0.5))  # outside
    n = f.normal(0.75, 0.25)
    assert math.isclose(n[2], 1.0 / math.sqrt(2.0), abs_tol=1e-6) and n[1] < 0.0
    xs = np.array([[0.5, 0.75], [0.25, 2.0]])
    ys = np.array([[0.5, 0.25], [0.75, 0.5]])
    hs = f.heights_at(xs, ys)
    assert hs.shape == (2, 2)
    assert np.allclose(hs[0], [0.5, 0.25]) and hs[1, 0] == f.height(0.25, 0.75)
    assert math.isnan(hs[1, 1])
    assert f.cell_of(0.5, 0.5) == (0, 0) and f.cell_of(-1.0, 0.5) is None


def test_holes_and_cell_materials():
    w = pycvc.world_geometry()
    asphalt, grass = w.surface_id("asphalt"), w.surface_id("grass")
    h = [0.0, 0.0, 0.0, 0.0, 0.0, float("nan"), 0.0, 0.0, 0.0]
    f = pycvc.height_field(3, 3, 0.0, 0.0, 1.0, 1.0, h, [asphalt, grass, asphalt, grass])
    assert math.isnan(f.heights()[1, 2]) and f.vertex_of(1, 2) == -1
    assert math.isnan(f.height(1.5, 0.5))  # a cell touching the hole
    assert f.height(0.5, 0.5) == 0.0
    cm = f.cell_materials()
    assert cm.dtype == np.uint16 and cm.shape == (2, 2)
    assert cm.tolist() == [[asphalt, grass], [asphalt, grass]]
    assert f.material(0.5, 1.5) == asphalt and f.material(1.5, 1.5) == grass
    f.set_cell_material(0, 0, grass)
    assert f.cell_material(0, 0) == grass
    expect_raises(lambda: f.set_cell_materials([70000, 0, 0, 0]), "a surface id over 65535")


def test_world_geometry_items_and_surfaces():
    w = pycvc.world_geometry()
    f = pycvc.height_field(3, 3, 0.0, 0.0, 1.0, 1.0, [1.0] * 9)
    w.add_terrain("terrain", f, surface=w.surface_id("grass"))
    roof = pycvc.geometry(pycvc.make_app())
    roof.add_vertices([0, 0, 0, 1, 0, 0, 0, 1, 0])
    roof.add_triangles([0, 1, 2])
    metal = w.surface_id("building_metal")
    w.add_mesh("roof", roof, surface=metal, position=(10.0, 0.0, 5.0))
    assert w.names() == ("terrain", "roof") and len(w) == 2
    assert w.kind("terrain") == "height_field" and w.kind("roof") == "mesh"
    assert w.contains("roof") and not w.contains("nothing")
    assert w.position_of("roof") == (10.0, 0.0, 5.0)
    assert w.rotation_of("roof") == (1.0, 0.0, 0.0, 0.0)
    assert w.roles_of("roof") == 7 and w.surface_of("roof") == metal
    b = w.bounds()
    assert b[0] == 0.0 and b[3] == 11.0 and b[5] == 5.0  # the placed roof
    assert list(w.surfaces_used()) == sorted([w.surface_id("grass"), metal])
    t = w.terrain()
    assert t.rows() == 3 and t.height(1.0, 1.0) == 1.0
    assert w.item_geometry("roof").num_triangles() == 1
    assert w.item_geometry("terrain").num_vertices() == 9
    asphalt = w.surface(w.surface_id("asphalt"))
    assert asphalt["name"] == "asphalt" and 0.0 < asphalt["mu"] <= 1.5
    assert len(asphalt["albedo"]) == 3 and w.surface_count() > 10
    assert w.volume_names() == ()
    expect_raises(lambda: w.add_mesh("roof", roof), "a name already used")
    expect_raises(lambda: w.add_mesh("bad", roof, surface=60000), "a surface outside the table")
    expect_raises(
        lambda: w.add_mesh("tilted", roof, rotation=(2.0, 0, 0, 0)), "a non-unit rotation"
    )
    expect_raises(lambda: w.terrain("roof"), "a mesh as a height field")
    expect_raises(lambda: w.item_geometry("nothing"), "an unknown item")
    expect_raises(lambda: w.surface_id("no_such_surface"), "an unknown surface")


def write_bundle(d, grid, bounds=(100.0, 200.0, 103.0, 202.0)):
    """terrain.json as the bundles store it: north first (grid[0] is max_y)."""
    rows, cols = len(grid), len(grid[0])
    with open(os.path.join(d, "terrain.json"), "w") as f:
        json.dump(
            {
                "bounds": dict(zip(("min_x", "min_y", "max_x", "max_y"), bounds)),
                "rows": rows,
                "cols": cols,
                "grid": grid,
            },
            f,
        )


NORTH_FIRST = [[30.0, 31.0, 32.0, 33.0], [20.0, 21.0, 22.0, 23.0], [10.0, 11.0, 12.0, 13.0]]


def test_city_bundle_terrain_is_flipped_once():
    with tempfile.TemporaryDirectory() as d:
        write_bundle(d, NORTH_FIRST)
        city = pycvc.load_city_bundle(d)
        assert not city.buildings_loaded and "buildings.glb" in city.buildings_error
        assert city.bounds2d() == (100.0, 200.0, 103.0, 202.0)
        f = city.terrain()
        # Row 0 is min y: terrain.json's LAST row.
        assert f.heights()[0].tolist() == NORTH_FIRST[-1]
        assert f.height(100.0, 200.0) == 10.0  # south-west
        assert f.height(100.0, 202.0) == 30.0  # north-west: terrain.json's grid[0][0]
        assert f.height(103.0, 202.0) == 33.0
        assert city.world().names() == ("terrain",)
        e = pycvc.read_terrain_extent(d)
        assert (e["rows"], e["cols"], e["min_x"], e["max_y"]) == (3, 4, 100.0, 202.0)
        # A bundle written south first, read as such: no flip.
        south = pycvc.load_city_bundle(d, terrain_rows="south_first").terrain()
        assert south.heights()[0].tolist() == NORTH_FIRST[0]
        expect_raises(lambda: pycvc.load_city_bundle(d, terrain_rows="east"), "a bad row order")
        expect_raises(lambda: pycvc.load_city_bundle(os.path.join(d, "none")), "no terrain.json")


def load_scenes():
    """pymod_gl/scenes.py, loaded from its file: it needs only pycvc for the terrain."""
    spec = importlib.util.spec_from_file_location(
        "scenes_under_test", os.path.join(HERE, "pymod_gl", "scenes.py")
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_scenes_sampler_is_the_height_field():
    scenes = load_scenes()
    assert not hasattr(scenes, "terrain_grid"), "terrain_grid retired: C++ reads terrain.json"
    with tempfile.TemporaryDirectory() as d:
        write_bundle(d, NORTH_FIRST)
        f = scenes.load_terrain(os.path.join(d, "terrain.json"))
        assert f.heights()[0].tolist() == NORTH_FIRST[-1]
        for path in (d, os.path.join(d, "terrain.json")):
            h = scenes.terrain_sampler(path)
            for x, y in ((100.25, 200.6), (101.5, 201.5), (102.9, 200.1)):
                assert h(x, y) == f.height(x, y)
            # Clamped at the edges, as before.
            assert h(90.0, 190.0) == f.height(100.0, 200.0)
            assert h(110.0, 210.0) == f.height(103.0, 202.0)


def test_real_bundle_matches_terrain_json():
    d = os.environ.get("CVC_CITY_BUNDLE", "")
    if not d or not os.path.exists(os.path.join(d, "terrain.json")):
        print("  skipped: set CVC_CITY_BUNDLE to a bundle directory (e.g. austin_south)")
        return
    with open(os.path.join(d, "terrain.json")) as fh:
        raw = json.load(fh)
    city = pycvc.load_city_bundle(d, land_cover=True)
    f = city.terrain()
    flipped = np.array(raw["grid"][::-1], dtype=np.float32)
    assert f.heights().shape == flipped.shape
    assert np.array_equal(f.heights(), flipped), "Python's terrain is C++'s, sample for sample"
    print(
        "  %s: %dx%d samples, buildings %s, land cover %s, %d surfaces in use"
        % (
            os.path.basename(d.rstrip("/\\")),
            f.rows(),
            f.cols(),
            city.buildings_loaded,
            city.land_cover_loaded,
            len(city.world().surfaces_used()),
        )
    )


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            print(name)
            fn()
    print("test_pycvc_world_geometry: OK")
    sys.exit(0)
