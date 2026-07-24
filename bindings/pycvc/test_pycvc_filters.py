"""Robust tests for pycvc's volume filters and geometry mesh-processing.

Covers the facade methods that wrap libcvc's in-place processing:

  Volume:  bilateral_filter, contrast_enhancement, anisotropic_diffusion,
           gdtv_filter  (cvc::voxels' denoise / enhance filters; cvc::volume
           extends voxels, so they mutate the voxel grid in place).
  Geometry: smoothing, quality_improve.

Every filter is checked for a numeric INVARIANT (variance / neighbour-difference
energy drops, mean preserved, range behaves as expected, values stay finite),
not exact floats. All randomness is seeded, so the assertions are deterministic.

If CUDA is available, a filter is run on both a host and a GPU-resident volume
built from the SAME seeded input; the results must match within tolerance,
proving the C++ methods auto-dispatch to their CUDA kernels on GPU data.
"""

import numpy as np
import pycvc

# Explicit app threaded through every op (no module-global).
app = pycvc.make_app()

SEED = 12345


# ── helpers ─────────────────────────────────────────────────────────


def _volume_from_field(field):
    """Build a Float Volume from a (nz, ny, nx) float array.

    set_float_grid wants a flat row-major (x fastest, then y, then z) buffer;
    field.ravel(order="C") on a (nz, ny, nx) array yields exactly that order
    (index = ((k*ny)+j)*nx + i), matching Volume.value(i, j, k) / grid()[k,j,i].
    """
    nz, ny, nx = field.shape
    vol = pycvc.volume(app)
    vol.set_float_grid(field.astype(np.float64).ravel(order="C").tolist(),
                       nx, ny, nz, 0, 0, 0, 1, 1, 1)
    return vol


def _grid(vol):
    """A float64 copy of the volume's current voxel grid (safe snapshot)."""
    return np.asarray(vol.grid(), dtype=np.float64).copy()


def _flat_noisy(n=12, base=50.0, noise_amp=6.0, seed=SEED):
    rng = np.random.default_rng(seed)
    return base + rng.normal(0.0, noise_amp, size=(n, n, n))


def _neighbour_energy(a):
    """Sum of squared differences between adjacent voxels along all 3 axes
    (a proxy for high-frequency / noise content)."""
    return float(
        np.sum(np.diff(a, axis=0) ** 2)
        + np.sum(np.diff(a, axis=1) ** 2)
        + np.sum(np.diff(a, axis=2) ** 2)
    )


def _mesh_from(verts, faces):
    g = pycvc.geometry(app)
    g.add_vertices(verts.astype(np.float64).ravel(order="C").tolist())
    g.add_triangles(faces.astype(np.int64).ravel(order="C").tolist())
    return g


def _noisy_grid_mesh(m=14, noise_amp=0.12, seed=SEED):
    """An (m x m) triangulated plane in z=0 with seeded per-vertex z noise.
    Vertex index = j*m + i, so the height field reshapes to z[j, i]."""
    rng = np.random.default_rng(seed)
    xs = np.linspace(0.0, 1.0, m)
    ys = np.linspace(0.0, 1.0, m)
    verts = np.empty((m * m, 3), dtype=np.float64)
    for j in range(m):
        for i in range(m):
            verts[j * m + i] = (xs[i], ys[j], rng.normal(0.0, noise_amp))
    faces = []
    for j in range(m - 1):
        for i in range(m - 1):
            a, b = j * m + i, j * m + i + 1
            c, d = (j + 1) * m + i, (j + 1) * m + i + 1
            faces.append((a, b, c))
            faces.append((b, d, c))
    return _mesh_from(verts, np.array(faces, dtype=np.int64)), m


def _laplacian_roughness(verts, m):
    """High-frequency roughness of the height field: sum over interior grid
    vertices of (z - mean(4 neighbours))^2. Invariant to global translation
    or curl, so it isolates denoising."""
    z = verts[:, 2].reshape(m, m)
    lap = z[1:-1, 1:-1] - 0.25 * (z[:-2, 1:-1] + z[2:, 1:-1] + z[1:-1, :-2] + z[1:-1, 2:])
    return float(np.sum(lap ** 2))


def _icosphere(subdiv=2, radius=1.0):
    """A closed, manifold icosphere (subdivided icosahedron) — a mesh the LBIE
    quality improver can operate on."""
    t = (1.0 + 5.0 ** 0.5) / 2.0
    verts = [[-1, t, 0], [1, t, 0], [-1, -t, 0], [1, -t, 0],
             [0, -1, t], [0, 1, t], [0, -1, -t], [0, 1, -t],
             [t, 0, -1], [t, 0, 1], [-t, 0, -1], [-t, 0, 1]]
    verts = [list(map(float, v)) for v in verts]
    faces = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
             (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
             (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
             (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    cache = {}

    def midpoint(a, b):
        key = (min(a, b), max(a, b))
        if key not in cache:
            va, vb = verts[a], verts[b]
            verts.append([(va[i] + vb[i]) / 2.0 for i in range(3)])
            cache[key] = len(verts) - 1
        return cache[key]

    for _ in range(subdiv):
        nf = []
        for a, b, c in faces:
            ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
            nf += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        faces = nf
    v = np.array(verts, dtype=np.float64)
    v *= radius / np.linalg.norm(v, axis=1, keepdims=True)
    return v, np.array(faces, dtype=np.int64)


def _tri_quality(verts, faces):
    """Per-triangle normalised quality in (0, 1]: 1 == equilateral, 0 ==
    degenerate.  q = 4*sqrt(3)*area / (sum of squared edge lengths)."""
    p = verts[faces]
    e0, e1, e2 = p[:, 1] - p[:, 0], p[:, 2] - p[:, 1], p[:, 0] - p[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(e0, -e2), axis=1)
    l2 = (e0 ** 2).sum(1) + (e1 ** 2).sum(1) + (e2 ** 2).sum(1)
    return 4.0 * np.sqrt(3.0) * area / np.maximum(l2, 1e-30)


# ── Volume filters ──────────────────────────────────────────────────


def test_bilateral_filter_denoises():
    vol = _volume_from_field(_flat_noisy())
    before = _grid(vol)
    vol.bilateral_filter(20.0, 1.5, 2)
    after = _grid(vol)
    assert np.all(np.isfinite(after))
    e0, e1 = _neighbour_energy(before), _neighbour_energy(after)
    assert e1 < e0, (e0, e1)
    assert after.var() < before.var()
    assert abs(after.mean() - before.mean()) < 0.05 * abs(before.mean())
    print(f"bilateral_filter: neighbour-energy {e0:.1f} -> {e1:.1f}, "
          f"var {before.var():.2f} -> {after.var():.2f}, "
          f"mean {before.mean():.3f} -> {after.mean():.3f}")


def test_bilateral_preserves_smooth_structure():
    # Smooth gaussian bump + seeded noise; bilateral should pull the field
    # CLOSER to the underlying smooth truth (denoise without wrecking structure).
    n = 16
    zz, yy, xx = np.mgrid[0:n, 0:n, 0:n]
    c = (n - 1) / 2.0
    r2 = (xx - c) ** 2 + (yy - c) ** 2 + (zz - c) ** 2
    base = 30.0 * np.exp(-r2 / (2.0 * (n / 4.0) ** 2)) + 20.0
    rng = np.random.default_rng(SEED)
    field = base + rng.normal(0.0, 5.0, size=(n, n, n))
    vol = _volume_from_field(field)
    vol.bilateral_filter(15.0, 1.5, 2)
    after = _grid(vol)
    rms_before = float(np.sqrt(np.mean((field - base) ** 2)))
    rms_after = float(np.sqrt(np.mean((after - base) ** 2)))
    assert np.all(np.isfinite(after))
    assert rms_after < rms_before, (rms_before, rms_after)
    print(f"bilateral (structure): RMS-to-truth {rms_before:.3f} -> {rms_after:.3f}")


def test_anisotropic_diffusion_denoises():
    vol = _volume_from_field(_flat_noisy())
    before = _grid(vol)
    vol.anisotropic_diffusion(15)
    after = _grid(vol)
    assert np.all(np.isfinite(after))
    e0, e1 = _neighbour_energy(before), _neighbour_energy(after)
    assert e1 < e0, (e0, e1)
    assert abs(after.mean() - before.mean()) < 0.05 * abs(before.mean())
    print(f"anisotropic_diffusion: neighbour-energy {e0:.1f} -> {e1:.1f}, "
          f"mean {before.mean():.3f} -> {after.mean():.3f}")


def test_contrast_enhancement_changes_range():
    # Low-contrast sawtooth in [40, 60): contrast enhancement redistributes
    # values (variance UP) while restoring the original value range.
    n = 10
    field = np.array([40.0 + (i % 20) for i in range(n * n * n)]).reshape(n, n, n)
    vol = _volume_from_field(field)
    before = _grid(vol)
    vol.contrast_enhancement(0.9)
    after = _grid(vol)
    assert np.all(np.isfinite(after))
    assert not np.allclose(after, before)
    assert after.var() > before.var(), (before.var(), after.var())
    rng_before = before.max() - before.min()
    rng_after = after.max() - after.min()
    assert abs(rng_after - rng_before) < 3.0, (rng_before, rng_after)
    print(f"contrast_enhancement: var {before.var():.2f} -> {after.var():.2f}, "
          f"range {rng_before:.2f} -> {rng_after:.2f} (preserved)")


def test_gdtv_filter_runs_and_denoises():
    vol = _volume_from_field(_flat_noisy())
    before = _grid(vol)
    vol.gdtv_filter(1.5, 0.3, 5, 0)  # q, lambda, iterations, 6-neighbour
    after = _grid(vol)
    assert np.all(np.isfinite(after))
    assert not np.array_equal(after, before)
    assert after.var() < before.var(), (before.var(), after.var())
    assert abs(after.mean() - before.mean()) < 2.0
    print(f"gdtv_filter: var {before.var():.2f} -> {after.var():.2f}, "
          f"mean {before.mean():.3f} -> {after.mean():.3f}")


def test_filter_cuda_dispatch_matches_host():
    if not pycvc.volume.cuda_available():
        print("[skip] test_filter_cuda_dispatch_matches_host: CUDA not available")
        return
    field = _flat_noisy(n=12)

    vol_h = _volume_from_field(field)
    vol_h.bilateral_filter(20.0, 1.5, 2)
    host = _grid(vol_h)

    vol_g = _volume_from_field(field)
    vol_g.enable_cuda()
    assert vol_g.on_gpu(), "enable_cuda() should make the volume GPU-resident"
    vol_g.bilateral_filter(20.0, 1.5, 2)
    # enable_cuda() REALLOCATED the buffer, so fetch the view AFTER migration.
    gpu = _grid(vol_g)
    vol_g.disable_cuda()

    assert np.all(np.isfinite(gpu))
    max_diff = float(np.abs(host - gpu).max())
    assert np.allclose(host, gpu, atol=1e-1, rtol=1e-3), f"max|diff|={max_diff}"
    print(f"CUDA dispatch: GPU bilateral matches host, max|diff|={max_diff:.3e}")


# ── Geometry mesh-processing ────────────────────────────────────────


def test_geometry_smoothing_moves_and_smooths():
    g, m = _noisy_grid_mesh()
    nv, nt = g.num_vertices(), g.num_triangles()
    before = g.vertices().copy()
    rough_before = _laplacian_roughness(before, m)
    for _ in range(3):
        g.smoothing(app, 0.5, False, False)  # delta, fix_boundary, geometric_flow=Laplacian
    after = np.asarray(g.vertices())
    # topology preserved
    assert g.num_vertices() == nv and g.num_triangles() == nt
    # vertices actually moved
    assert float(np.max(np.abs(after - before))) > 1e-6
    assert np.all(np.isfinite(after))
    # surface got smoother (high-frequency height roughness dropped)
    rough_after = _laplacian_roughness(np.asarray(after), m)
    assert rough_after < rough_before, (rough_before, rough_after)
    print(f"smoothing: {nv} verts / {nt} tris preserved, "
          f"roughness {rough_before:.4f} -> {rough_after:.4f}")


def test_geometry_quality_improve_runs():
    # Mildly noisy icosphere so the quality metric has room to improve.
    verts, faces = _icosphere(subdiv=2)
    rng = np.random.default_rng(SEED)
    noisy = verts + rng.normal(0.0, 0.03, size=verts.shape)
    g = _mesh_from(noisy, faces)
    nv, nt = g.num_vertices(), g.num_triangles()
    try:
        g.quality_improve(2, 1)  # iterations, GEO_FLOW
    except RuntimeError as e:
        if "mesher disabled" in str(e):
            print("[skip] test_geometry_quality_improve_runs: mesher disabled in build")
            return
        raise
    out = np.asarray(g.vertices())
    assert g.num_vertices() > 0 and g.num_triangles() > 0
    assert np.all(np.isfinite(out))
    if g.num_vertices() == nv and g.num_triangles() == nt:
        q_before = float(_tri_quality(noisy, faces).min())
        q_after = float(_tri_quality(out, faces).min())
        # topology preserved -> min triangle quality improved or held
        assert q_after >= q_before - 1e-4, (q_before, q_after)
        print(f"quality_improve: topology preserved, "
              f"min-tri-quality {q_before:.4f} -> {q_after:.4f}")
    else:
        print(f"quality_improve: remeshed {nv}/{nt} -> "
              f"{g.num_vertices()}/{g.num_triangles()} (valid, finite)")


if __name__ == "__main__":
    print("=== pycvc filter / mesh-processing tests ===")
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("pycvc filter tests: OK")
