"""pycvc_gl: the Lab draws the world's own height field (G4b of
docs/roadmap/WORLD-GEOMETRY-ROADMAP.md).

  1. Lab.add_height_field draws a pycvc.height_field's own mesh, and Lab.add_terrain
     goes through it, so the drawn surface is the one bodies rest on;
  2. terrain_mesh splits cells south-west to north-east, as cvc::height_field does;
  3. scenes.load_geometry_bundle loads through pycvc.load_city_bundle and returns a
     sampler that is the drawn field's height.
Headless: nothing is rendered.
"""

from __future__ import annotations

import json
import os
import sys
import tempfile

import pycvc
import pycvc_gl  # noqa: F401  (the package the Lab and scenes live in)
from pycvc_gl import scenes
from pycvc_gl.lab import Lab, terrain_mesh


def test_add_height_field_draws_the_fields_mesh():
    lab = Lab()
    field = pycvc.height_field(3, 4, 0.0, 0.0, 1.0, 1.0, [float(i) for i in range(12)])
    lab.add_height_field("ground", field, color=(0.3, 0.5, 0.2))
    assert lab._scene.getGraphics("ground") is not None
    lab.add_terrain([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]], (0.0, 0.0, 2.0, 2.0))
    assert lab._scene.getGraphics("terrain") is not None


def test_terrain_mesh_splits_like_the_height_field():
    verts, tris = terrain_mesh([[0.0, 0.0], [0.0, 1.0]], (0.0, 0.0, 1.0, 1.0))
    assert len(verts) == 12 and len(tris) == 6
    # South-west (0) to north-east (3): both triangles hold the diagonal.
    assert tris == [0, 1, 3, 0, 3, 2]
    field = pycvc.height_field(2, 2, 0.0, 0.0, 1.0, 1.0, [0.0, 0.0, 0.0, 1.0])
    # The mesh's own triangles, as the field reports them, share that diagonal.
    assert abs(field.height(0.5, 0.5) - 0.5) < 1e-6


def test_load_geometry_bundle_samples_what_it_draws():
    with tempfile.TemporaryDirectory() as d:
        grid = [[3.0, 3.0, 3.0], [2.0, 2.0, 2.0], [1.0, 1.0, 1.0]]  # north first
        with open(os.path.join(d, "terrain.json"), "w") as f:
            json.dump(
                {
                    "bounds": {"min_x": 0.0, "min_y": 0.0, "max_x": 2.0, "max_y": 2.0},
                    "rows": 3,
                    "cols": 3,
                    "grid": grid,
                },
                f,
            )
        lab = Lab()
        h = scenes.load_geometry_bundle(lab, d)  # no buildings.glb: terrain only
        assert lab._scene.getGraphics("terrain") is not None
        assert lab._scene.getGraphics("buildings") is None
        field = scenes.load_terrain(d)
        for x, y in ((0.0, 0.0), (1.0, 1.0), (0.3, 1.7), (2.0, 2.0)):
            assert h(x, y) == field.height(x, y)
        assert h(0.0, 0.0) == 1.0 and h(0.0, 2.0) == 3.0  # row 0 at min y


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            print(name)
            fn()
    print("test_pycvc_gl_world_geometry: OK")
    sys.exit(0)
