"""pycvc skeleton acceptance tests: numpy -> libcvc -> numpy round trips.

The G5 acceptance bar (cvc-engagement-docs/planning/near-term-goals-2026-07-15.md):
a numpy float array round-trips through a cvc::volume, gets bilateral-filtered
(and anisotropic-diffused), returns to numpy with the same shape and
changed-but-finite values, and vol_normalize clamps to [0, 1].

Run manually against a CVC_BUILD_PYTHON=ON build tree:

    PYTHONPATH=<build-dir>/python python -m pytest src/pycvc/tests/test_roundtrip.py -v

Array convention: numpy shape (Z, Y, X) in C order maps directly onto the
libcvc voxel layout (x fastest).
"""

import numpy as np
import pytest

pycvc = pytest.importorskip("pycvc")


def _noise(shape, seed=42):
    rng = np.random.default_rng(seed)
    return rng.random(shape, dtype=np.float32)


def test_volume_roundtrip_identity():
    a = _noise((32, 32, 8))
    v = pycvc.volume(a)
    assert v.shape == (32, 32, 8)
    assert (v.XDim(), v.YDim(), v.ZDim()) == (8, 32, 32)
    assert v.voxelType() == pycvc.Float
    back = np.asarray(v)
    assert back.shape == a.shape
    assert back.dtype == np.float32
    np.testing.assert_array_equal(back, a)


def test_bilateral_filter_roundtrip():
    a = _noise((32, 32, 8))
    v = pycvc.volume(a)
    v.bilateralFilter(200.0, 1.5, 2)
    out = np.asarray(v)
    assert out.shape == a.shape
    assert np.all(np.isfinite(out))
    assert not np.array_equal(out, a)  # values changed...
    assert out.std() < a.std()  # ...and the field got smoother


def test_anisotropic_diffusion_roundtrip():
    a = _noise((32, 32, 8), seed=7)
    v = pycvc.volume(a)
    v.anisotropicDiffusion(5)
    out = np.asarray(v)
    assert out.shape == a.shape
    assert np.all(np.isfinite(out))
    assert not np.array_equal(out, a)
    assert out.std() < a.std()


def test_vol_normalize_clamps_to_unit_interval():
    a = (_noise((16, 16, 4), seed=3) * 10.0 - 5.0).astype(np.float32)
    v = pycvc.volume(a)
    n = pycvc.vol_normalize(v, 0.0, 1.0)
    out = np.asarray(n)
    assert out.shape == a.shape
    assert out.min() >= 0.0 and out.max() <= 1.0
    assert out.min() == pytest.approx(0.0, abs=1e-6)
    assert out.max() == pytest.approx(1.0, abs=1e-6)
    # normalize is a monotone remap: argmin/argmax preserved
    assert np.unravel_index(out.argmax(), out.shape) == np.unravel_index(a.argmax(), a.shape)


def test_bounding_box_spacing_and_interpolate():
    a = _noise((8, 16, 4), seed=11)  # (Z, Y, X) -> dims (4, 16, 8)
    bbox = pycvc.bounding_box(0.0, 0.0, 0.0, 3.0, 15.0, 7.0)
    v = pycvc.volume(a, bbox)
    assert v.XMin() == 0.0 and v.XMax() == 3.0
    assert (v.XSpan(), v.YSpan(), v.ZSpan()) == (1.0, 1.0, 1.0)
    # trilinear interpolation at an exact grid point == the array value
    assert v.interpolate(2.0, 3.0, 4.0) == pytest.approx(float(a[4, 3, 2]), abs=1e-6)


def test_smoothed_risk_field_pipeline():
    """The r-tilde pipeline: noise -> bilateral -> normalize -> [0,1] numpy."""
    a = _noise((16, 32, 32), seed=5)
    v = pycvc.volume(a)
    v.bilateralFilter()
    r = pycvc.vol_normalize(v, 0.0, 1.0)
    out = np.asarray(r)
    assert out.shape == a.shape
    assert np.all(np.isfinite(out))
    assert out.min() >= 0.0 and out.max() <= 1.0
    # the remap must stretch the *filtered* field to the full unit interval
    # (guards against libcvc's stale cached min/max after in-place filters —
    # the binding invalidates the cache after each filter call)
    assert out.min() == pytest.approx(0.0, abs=1e-6)
    assert out.max() == pytest.approx(1.0, abs=1e-6)


@pytest.mark.skipif(not hasattr(pycvc, "sdf"), reason="libcvc built without CVC_ENABLE_SDF")
def test_mesh_to_sdf_to_numpy():
    """Mesh -> SDF -> numpy round trip: sign change across a sphere surface."""
    sphere = pycvc.generate_sphere(0.0, 0.0, 0.0, 1.0, 16, 8)
    assert sphere.num_points() > 0 and sphere.num_tris() > 0

    # mesh points/tris are visible from numpy...
    pts = sphere.points_to_numpy()
    tris = sphere.tris_to_numpy()
    assert pts.shape[1] == 3 and tris.shape[1] == 3

    # ...and a geometry can be built back from numpy arrays
    geom = pycvc.geometry()
    geom.set_mesh(pts, tris)
    assert geom.num_points() == sphere.num_points()
    assert geom.num_tris() == sphere.num_tris()

    bbox = pycvc.bounding_box(-1.5, -1.5, -1.5, 1.5, 1.5, 1.5)
    phi = pycvc.sdf(geom, 16, 16, 16, bbox)
    out = np.asarray(phi)
    assert out.shape == (16, 16, 16)
    assert np.all(np.isfinite(out))
    # signed: inside and outside must differ in sign
    assert out.min() < 0.0 < out.max()


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
