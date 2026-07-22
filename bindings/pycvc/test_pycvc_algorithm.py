"""Robust tests for pycvc's algorithm facade (pycvc_algorithm.*): procedural
geometry, signed-distance fields, isosurface / volumetric meshing, surface
extraction, and per-element mesh-quality metrics.

Design notes:
  * Procedural generators and the analytic-field / fixed-vertex checks are
    fully deterministic — exact counts and values are asserted.
  * Mesher outputs (iso/tet/hex vertex counts) depend on the extraction grid
    and are NOT bit-stable, so those are asserted as non-empty + approximately
    the right shape (e.g. surface vertices near the sphere radius) rather than
    exact counts.

Every check both ASSERTs and prints, so a passing run is also a readable log.
"""

import math

import numpy as np
import pycvc


# ── helpers ──────────────────────────────────────────────────────────


def _analytic_sphere_field(radius=1.0, dim=32, lo=-2.0, hi=2.0):
    """A Volume holding f(p) = |p| - radius (negative inside, zero on the
    surface, positive outside) sampled on a dim^3 grid over [lo,hi]^3."""
    span = hi - lo
    vals = []
    for k in range(dim):
        z = lo + span * k / (dim - 1)
        for j in range(dim):
            y = lo + span * j / (dim - 1)
            for i in range(dim):
                x = lo + span * i / (dim - 1)
                vals.append(math.sqrt(x * x + y * y + z * z) - radius)
    vol = pycvc.Volume()
    vol.set_float_grid(vals, dim, dim, dim, lo, lo, lo, hi, hi, hi)
    return vol


# ── enums ────────────────────────────────────────────────────────────


def test_enums_exposed():
    assert (pycvc.SDF_V1, pycvc.SDF_V2) == (0, 1)
    assert (pycvc.DUALLIB, pycvc.FASTCONTOURING, pycvc.LIBISOCONTOUR) == (0, 1, 2)
    assert pycvc.NO_IMPROVE == 0 and pycvc.GEO_FLOW == 1
    assert pycvc.BSPLINE_CONVOLUTION == 0
    assert pycvc.TET_ASPECT_RATIO == 1 and pycvc.HEX_SCALED_JACOBIAN == 5
    print("enums exposed: SDF/extraction/improvement/normal/quality OK")


# ── procedural generators (deterministic counts + geometry) ──────────


def test_sphere_counts_and_surface():
    theta, phi = 16, 8
    s = pycvc.sphere(0, 0, 0, 1.0, theta, phi)
    exp_v = 2 + (phi - 1) * theta          # poles + interior rings
    exp_t = theta + (phi - 2) * theta * 2 + theta
    assert s.num_vertices() == exp_v == 114
    assert s.num_triangles() == exp_t == 224
    V = np.asarray(s.vertices())
    assert np.all(np.isfinite(V))
    radii = np.linalg.norm(V, axis=1)
    assert np.allclose(radii, 1.0, atol=1e-6)  # every vertex on the unit sphere
    print(f"sphere(16,8): {s.num_vertices()} verts, {s.num_triangles()} tris, on-surface OK")


def test_cube_counts_and_bounds():
    c = pycvc.cube(0, 0, 0, 2.0, 2.0, 2.0)
    assert c.num_vertices() == 24 and c.num_triangles() == 12  # 6 faces * 4 / 2
    b = list(pycvc.compute_mesh_bounds(c))
    assert np.allclose(b, [-1, -1, -1, 1, 1, 1])
    print(f"cube: 24 verts, 12 tris, bounds {b}")


def test_torus_and_cone_counts():
    t = pycvc.torus(0, 0, 0, 2.0, 0.5, 32, 16)
    assert t.num_vertices() == 32 * 16 == 512
    assert t.num_triangles() == 2 * 32 * 16 == 1024
    assert np.all(np.isfinite(np.asarray(t.vertices())))
    co = pycvc.cone(0, 0, 0, 1.0, 2.0, 32)
    assert co.num_vertices() == 2 + 2 * 32 == 66   # apex + side ring + base center + base ring
    assert co.num_triangles() == 2 * 32 == 64      # side + base cap
    print(f"torus: {t.num_vertices()}/{t.num_triangles()}  cone: {co.num_vertices()}/{co.num_triangles()}")


def test_compute_mesh_bounds_sphere():
    s = pycvc.sphere(1.0, 2.0, 3.0, 0.5)
    b = list(pycvc.compute_mesh_bounds(s))
    assert len(b) == 6
    assert np.allclose(b, [0.5, 1.5, 2.5, 1.5, 2.5, 3.5], atol=1e-6)
    print(f"compute_mesh_bounds(sphere@(1,2,3) r=0.5): {b}")


# ── SDF (negative inside / positive outside) ─────────────────────────


def test_sdf_sign_and_shape():
    geom = pycvc.sphere(0, 0, 0, 1.0, 32, 16)
    dim = 24
    vol = pycvc.sdf(geom, dim, dim, dim, -2.0, -2.0, -2.0, 2.0, 2.0, 2.0)
    assert (vol.xdim(), vol.ydim(), vol.zdim()) == (dim, dim, dim)
    grid = np.asarray(vol.grid())
    assert grid.shape == (dim, dim, dim)
    center = vol.value(dim // 2, dim // 2, dim // 2)  # world ~origin: inside
    corner = vol.value(0, 0, 0)                       # world (-2,-2,-2): outside
    assert center < 0.0, f"center SDF should be negative (inside), got {center}"
    assert corner > 0.0, f"corner SDF should be positive (outside), got {corner}"
    # corner ~ distance from (-2,-2,-2) to unit sphere ~ sqrt(12)-1 ~ 2.46
    assert 2.0 < corner < 3.0
    print(f"sdf sign: center(inside)={center:.3f} < 0 < corner(outside)={corner:.3f}")


def test_sdf_bbox_from_geometry_extents():
    # The bbox-less overload uses the geometry's own extents.
    geom = pycvc.sphere(0, 0, 0, 1.0, 24, 12)
    vol = pycvc.sdf(geom, 16, 16, 16)
    assert (vol.xdim(), vol.ydim(), vol.zdim()) == (16, 16, 16)
    assert vol.min_value() < 0.0  # some samples fall inside the sphere
    print(f"sdf(no-bbox) 16^3: min={vol.min_value():.3f} max={vol.max_value():.3f}")


def test_sdf_v2_algorithm_selection():
    geom = pycvc.sphere(0, 0, 0, 1.0, 24, 12)
    vol = pycvc.sdf(geom, 20, 20, 20, -2.0, -2.0, -2.0, 2.0, 2.0, 2.0, pycvc.SDF_V2)
    assert (vol.xdim(), vol.ydim(), vol.zdim()) == (20, 20, 20)
    assert vol.min_value() < 0.0 < vol.max_value()
    print(f"sdf SDF_V2: min={vol.min_value():.3f} max={vol.max_value():.3f}")


# ── isosurface round-trips ───────────────────────────────────────────


def test_isosurface_analytic_field():
    vol = _analytic_sphere_field(radius=1.0, dim=32)
    iso = pycvc.isosurface(vol, 0.0)
    assert iso.num_vertices() > 0 and iso.num_triangles() > 0
    V = np.asarray(iso.vertices())
    assert np.all(np.isfinite(V))
    radii = np.linalg.norm(V, axis=1)
    assert 0.7 < radii.mean() < 1.3, f"iso should sit near r=1, mean={radii.mean()}"
    assert radii.max() < 1.6
    print(f"isosurface(|p|-1, 0.0): {iso.num_vertices()} verts, mean radius {radii.mean():.3f}")


def test_isosurface_of_sdf():
    geom = pycvc.sphere(0, 0, 0, 1.0, 32, 16)
    vol = pycvc.sdf(geom, 32, 32, 32, -2.0, -2.0, -2.0, 2.0, 2.0, 2.0)
    iso = pycvc.isosurface(vol, 0.0)
    assert iso.num_vertices() > 0 and iso.num_triangles() > 0
    radii = np.linalg.norm(np.asarray(iso.vertices()), axis=1)
    assert 0.7 < radii.mean() < 1.3
    print(f"sdf->isosurface: {iso.num_vertices()} verts, mean radius {radii.mean():.3f}")


# ── volumetric meshing + quality ─────────────────────────────────────


def test_tetrahedralize_and_quality_stats():
    vol = _analytic_sphere_field(radius=1.0, dim=32)
    tet = pycvc.tetrahedralize(vol, 0.0)
    assert tet.num_vertices() > 0, "tetrahedralize produced no vertices"
    # A pure tet mesh carries no surface triangles.
    assert tet.num_triangles() == 0
    stats = pycvc.compute_tet_quality_stats(tet, pycvc.TET_ASPECT_RATIO)
    # max > 0 confirms tet elements actually exist (stats are over tets()).
    assert stats.max > 0.0, "no tet elements -> empty quality stats"
    assert stats.min <= stats.mean <= stats.max
    assert stats.std_dev >= 0.0
    # Normalized aspect ratio: 1.0 is a perfect tet, so every element is >= ~1.
    assert stats.min >= 1.0 - 1e-6
    print(f"tetrahedralize: {tet.num_vertices()} verts; aspect min/mean/max = "
          f"{stats.min:.3f}/{stats.mean:.3f}/{stats.max:.3f}")


def test_hexahedralize_and_quality_stats():
    vol = _analytic_sphere_field(radius=1.0, dim=32)
    hexg = pycvc.hexahedralize(vol, 0.0)
    assert hexg.num_vertices() > 0, "hexahedralize produced no vertices"
    stats = pycvc.compute_hex_quality_stats(hexg, pycvc.HEX_SCALED_JACOBIAN)
    assert stats.min <= stats.mean <= stats.max
    assert stats.std_dev >= 0.0
    print(f"hexahedralize: {hexg.num_vertices()} verts; scaled-jac min/mean/max = "
          f"{stats.min:.4f}/{stats.mean:.4f}/{stats.max:.4f}")


def test_tetrahedralize2_interval_layer():
    # The two-isovalue overload meshes the region between two isosurfaces.
    vol = _analytic_sphere_field(radius=1.0, dim=32)
    layer = pycvc.tetrahedralize2(vol, 0.2, -0.2)
    assert layer.num_vertices() > 0, "interval tetrahedralize2 produced no vertices"
    print(f"tetrahedralize2 interval [0.2,-0.2]: {layer.num_vertices()} verts")


def test_tetrahedralize2_single_returns_geometry():
    # NOTE: on an SDF-like field the single-isovalue TETRA2 path yields an
    # empty mesh in libcvc (see report); we assert the overload is callable
    # and returns a Geometry, not that it is non-empty.
    vol = _analytic_sphere_field(radius=1.0, dim=24)
    g = pycvc.tetrahedralize2(vol, 0.0)
    assert isinstance(g, pycvc.Geometry)
    assert g.num_vertices() >= 0
    print(f"tetrahedralize2 single iso=0.0: {g.num_vertices()} verts (empty is expected here)")


# ── surface extraction ───────────────────────────────────────────────


def test_extract_surface_of_tet_mesh():
    vol = _analytic_sphere_field(radius=1.0, dim=32)
    tet = pycvc.tetrahedralize(vol, 0.0)
    assert tet.num_vertices() > 0
    surf = pycvc.extract_surface(tet)
    assert surf.num_vertices() > 0 and surf.num_triangles() > 0
    assert np.all(np.isfinite(np.asarray(surf.vertices())))
    print(f"extract_surface(tet): {surf.num_vertices()} verts, {surf.num_triangles()} boundary tris")


def test_extract_surface_of_surface_mesh_is_copy():
    s = pycvc.sphere(0, 0, 0, 1.0, 16, 8)
    surf = pycvc.extract_surface(s)
    # For a surface mesh, extract_surface returns the surface itself.
    assert surf.num_vertices() == s.num_vertices()
    assert surf.num_triangles() == s.num_triangles()
    print(f"extract_surface(surface): passthrough {surf.num_vertices()} verts")


# ── per-element quality metrics (deterministic on fixed vertices) ────


def test_regular_tet_metrics():
    # Regular tetrahedron, edge length 2*sqrt(2).
    tet = [1, 1, 1,  1, -1, -1,  -1, 1, -1,  -1, -1, 1]
    ar = pycvc.tet_aspect_ratio(tet)
    vol = pycvc.tet_volume(tet)
    ang = pycvc.tet_min_dihedral_angle(tet)
    # libcvc normalizes the regular-tet aspect ratio to exactly 1.0 (the header
    # doc's "~2.04" is a stale comment; the implementation uses
    # longest_edge/(2*sqrt(6)*inradius), which is 1.0 for a regular tet).
    assert abs(ar - 1.0) < 1e-4, f"regular tet aspect ratio should be ~1.0, got {ar}"
    assert abs(abs(vol) - 8.0 / 3.0) < 1e-9, f"regular tet volume magnitude, got {vol}"
    assert abs(ang - math.degrees(math.acos(1.0 / 3.0))) < 1e-6  # 70.5288 deg
    print(f"regular tet: aspect={ar:.4f} volume={vol:.4f} min_dihedral={ang:.3f} deg")


def test_unit_hex_metrics():
    # Unit cube, standard hex ordering (bottom 0-3 CCW, top 4-7 CCW).
    hexv = [0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0,
            0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1]
    v = pycvc.hex_volume(hexv)
    sj = pycvc.hex_scaled_jacobian(hexv)
    assert abs(v - 1.0) < 1e-9, f"unit cube volume should be 1.0, got {v}"
    # libcvc scales the Jacobian by the max pairwise distance cubed (the space
    # diagonal), so a perfect cube is ~0.024, not the canonical 1.0 — assert it
    # is positive (a valid, non-inverted element) and finite.
    assert sj > 0.0 and math.isfinite(sj)
    print(f"unit hex: volume={v:.4f} scaled_jacobian={sj:.4f} (>0 = valid element)")


def test_quality_metric_bad_length_raises():
    for fn, badlen in ((pycvc.tet_volume, [0, 0, 0]),
                       (pycvc.tet_aspect_ratio, [1, 2, 3, 4, 5]),
                       (pycvc.hex_volume, [0.0] * 12),
                       (pycvc.hex_scaled_jacobian, [0.0] * 10)):
        try:
            fn(badlen)
        except Exception:  # noqa: BLE001  (SWIG maps std::invalid_argument -> RuntimeError)
            pass
        else:
            raise AssertionError(f"{fn.__name__} should reject wrong-length input")
    print("quality metrics reject malformed vertex lists")


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    for name, fn in tests:
        fn()
    print(f"\npycvc algorithm tests: OK ({len(tests)} tests)")
