"""pycvc in-library data-prep tests — volume + geometry ops a training/twin
script needs before handing data to a network or a renderer.

These wrap voxels::map/fill/resize/sub (re-exposed on the volume proxy under
distinct names, since the base %ignore reaches the derived %extend) and the
geometry normals/extents/orientation surface that was previously write-only or
opaque. All headless, all read back through the zero-copy grid()/vertices()
numpy views.
"""

import math

import numpy as np
import pycvc

app = pycvc.make_app()


def _vol(vals, nx, ny, nz):
    v = pycvc.volume(app)
    v.set_float_grid(list(vals), nx, ny, nz, 0, 0, 0, 1, 1, 1)
    return v


# ── volume data-prep ─────────────────────────────────────────────────────────


def test_normalize_maps_data_range():
    v = _vol([float(i) for i in range(8)], 2, 2, 2)  # values 0..7
    v.normalize(0.0, 1.0)
    g = v.grid()
    assert math.isclose(float(g.min()), 0.0, abs_tol=1e-6)
    assert math.isclose(float(g.max()), 1.0, abs_tol=1e-6)
    print("  ok: normalize remaps the data range to [lo, hi]")


def test_fill_value_sets_every_voxel():
    v = _vol([float(i) for i in range(8)], 2, 2, 2)
    v.fill_value(3.5)
    assert np.allclose(v.grid(), 3.5)
    print("  ok: fill_value sets every voxel to the constant")


def test_resample_resizes_in_place():
    v = _vol([1.0] * 8, 2, 2, 2)
    v.resample(4, 3, 5)
    assert v.grid().shape == (5, 3, 4)  # grid() is (nz, ny, nx)
    print("  ok: resample resizes the grid in place")


def test_crop_returns_new_leaves_source_intact():
    v = _vol([float(i) for i in range(8)], 2, 2, 2)
    sub = v.crop(0, 0, 0, 1, 1, 1)
    assert sub.grid().shape == (1, 1, 1)
    assert v.grid().shape == (2, 2, 2), "crop must not mutate the source"
    print("  ok: crop returns a new sub-grid and leaves the source intact")


# ── geometry normals / extents / orientation ─────────────────────────────────


def _tri():
    g = pycvc.geometry(app)
    g.add_vertices([0, 0, 0, 2, 0, 0, 0, 3, 0])
    g.add_triangle(0, 1, 2)
    return g


def test_normals_round_trip():
    g = _tri()
    g.set_normals([0.0, 0.0, 1.0] * 3)
    n = g.get_normals()
    assert len(n) == 9
    assert np.allclose(n, [0.0, 0.0, 1.0] * 3)
    # bad length raises, like set_colors
    try:
        g.set_normals([0.0, 0.0, 1.0])
    except Exception:
        pass
    else:
        raise AssertionError("set_normals with wrong length should raise")
    print("  ok: get_normals/set_normals round-trip; wrong length raises")


def test_compute_then_read_normals():
    g = _tri()
    g.compute_normals()  # already bound; now the result is readable
    n = g.get_normals()
    assert len(n) == 9 and all(math.isfinite(x) for x in n)
    print("  ok: compute_normals result is now readable via get_normals")


def test_invert_normals_flips():
    g = _tri()
    g.set_normals([0.0, 0.0, 1.0] * 3)
    g.invert_normals()
    assert np.allclose(g.get_normals(), [0.0, 0.0, -1.0] * 3)
    print("  ok: invert_normals negates the normals")


def test_extents_bbox():
    g = _tri()  # x in [0,2], y in [0,3], z == 0
    e = g.extents()
    assert len(e) == 6
    minx, miny, minz, maxx, maxy, maxz = e
    assert math.isclose(minx, 0.0) and math.isclose(maxx, 2.0)
    assert math.isclose(miny, 0.0) and math.isclose(maxy, 3.0)
    print("  ok: geometry.extents() -> (minx..maxz), like model.extents()")


def test_merge_and_reorient_and_tri_surface_callable():
    g = _tri()
    n0 = g.num_vertices()
    other = _tri()
    g.merge(other)  # self-returning geometry&, like compute_normals
    assert g.num_vertices() == n0 + other.num_vertices()
    g.reorient()  # winding fix; just exercise the binding
    ts = g.tri_surface()  # returns a new geometry by value
    assert ts is not None and ts.num_vertices() >= 0
    print("  ok: merge grows vertex count; reorient/tri_surface callable")


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("pycvc data-prep tests: OK")
