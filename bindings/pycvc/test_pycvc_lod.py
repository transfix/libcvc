"""pycvc LOD surface -- cvc::simplify, cvc::lod pyramids, selection math, tiles
and the scene.cvch5 store, driven from Python. Headless; synthetic data only.

What is proved here, beyond "the names exist":

  * round trips: a pyramid written to an in-memory scene.cvch5 comes back from
    the BYTES (trusted, and through open_verified with a hashlib SHA-256)
    identical to what was built -- and a file-backed container's to_blob() is
    the file's bytes;
  * the out-parameters arrive as return values (simplify -> (mesh, result)),
    and pooled builds match serial ones (content_hash of every rung);
  * the long calls RELEASE THE GIL: another Python thread keeps running for the
    whole of build_mesh_pyramid / simplify / simplify_progressive /
    build_tiled_pyramids / partition_parts / content_hash /
    build_image_pyramid and, in the store, write_mesh_pyramid / to_blob /
    scene_reader(bytes) / open_verified / read_mesh_pyramid / bake_mesh_asset,
    measured ONE C++ call at a time as the largest gap between that thread's
    ticks inside the call window (index / has are metadata walks too short to
    time; they release the GIL through the same guard);
  * Python callbacks run under the GIL from pool workers, and an exception a
    callback raises comes back out of the call that started it, type intact;
  * C++ failures map to the natural Python exception: ValueError for bad
    arguments, IndexError for a bad rung, OSError for HDF5/store failures,
    RuntimeError for a SHA-256 mismatch, TypeError for a non-bytes blob.

Runs under pytest or as a plain script (ctest: pycvc_lod).
"""

import gc
import hashlib
import math
import os
import tempfile
import threading
import time

import numpy as np

import pycvc

app = pycvc.make_app()


# ── synthetic meshes ─────────────────────────────────────────────────────────


def _terrain(n, x0=0.0, y0=0.0, size=100.0, amp=4.0):
    """An n x n heightfield over a `size` square with smooth relief, so
    decimation really removes detail (and really costs error)."""
    g = pycvc.geometry(app)
    xs = np.linspace(x0, x0 + size, n + 1)
    ys = np.linspace(y0, y0 + size, n + 1)
    X, Y = np.meshgrid(xs, ys)
    Z = 6.0 + amp * np.sin(0.21 * X) * np.cos(0.17 * Y) + 0.5 * amp * np.sin(0.53 * (X + Y))
    g.add_vertices(np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1).ravel().tolist())
    i, j = np.meshgrid(np.arange(n), np.arange(n))
    a = (j * (n + 1) + i).ravel()
    b, c = a + 1, a + n + 1
    d = c + 1
    g.add_triangles(np.stack([a, b, d, a, d, c], axis=1).ravel().tolist())
    return g


def _box(cx, cy, w, h, z0=0.0):
    """A closed axis-aligned box (12 triangles), base at z0."""
    g = pycvc.geometry(app)
    for k in range(8):
        g.add_vertex(cx + (w / 2 if k & 1 else -w / 2), cy + (w / 2 if k & 2 else -w / 2),
                     z0 + (h if k & 4 else 0.0))
    faces = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4),
             (2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5)]
    g.add_triangles([i for f in faces for i in f])
    return g


def _hashes(pyr):
    return [pycvc.content_hash(pyr.rung(k)) for k in range(len(pyr))]


def _same_ladder(a, b):
    """Rung for rung the same surface: identical vertex arrays and triangle
    counts, and a sampled Hausdorff distance of exactly 0. (Not content_hash:
    the scene.cvch5 layout stores points/tris/uv/colors but not normals, and a
    coarse rung carries freshly computed ones, which the hash counts.)"""
    assert len(a) == len(b) and list(a.world_error_m) == list(b.world_error_m)
    for k in range(len(a)):
        ra, rb = a.rung(k), b.rung(k)
        assert ra.num_triangles() == rb.num_triangles(), k
        assert np.array_equal(ra.vertices(), rb.vertices()), k
        assert pycvc.sampled_hausdorff(ra, rb) == 0.0, k
    return True


def _expect(exc_type, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc_type as e:
        return e
    raise AssertionError("%s did not raise %s" % (getattr(fn, "__name__", fn), exc_type.__name__))


# ── simplify ─────────────────────────────────────────────────────────────────


def test_simplify_returns_mesh_and_result():
    g = _terrain(32)
    p = pycvc.simplify_params()
    p.target_tris = 500
    assert p.target_tris == 500 and p.weld_seams and p.preserve_boundary
    out, res = pycvc.simplify(g, p)
    assert isinstance(out, pycvc.geometry) and isinstance(res, pycvc.simplify_result)
    assert res.in_tris == 2048 and res.out_tris == out.num_triangles() <= 500
    assert res.collapses > 0 and res.world_error > 0.0 and not res.hit_error_limit
    # keyword arguments + an owned pool give the identical mesh
    out2, res2 = pycvc.simplify(mesh=g, params=p, pool=pycvc.thread_pool(2))
    assert pycvc.content_hash(out2) == pycvc.content_hash(out)
    assert res2.world_error == res.world_error
    # the result's error IS the sampled Hausdorff distance
    assert math.isclose(pycvc.sampled_hausdorff(g, out), res.world_error, rel_tol=1e-12)
    assert pycvc.sampled_hausdorff(g, g) == 0.0
    # the source is untouched
    assert g.num_triangles() == 2048
    print("  ok: simplify -> (geometry, simplify_result), kwargs, pool, sampled_hausdorff")


def test_simplify_progressive_matches_simplify():
    g = _terrain(32)
    targets = [1000, 300, 100]
    rungs, results = pycvc.simplify_progressive(g, targets)
    assert len(rungs) == len(results) == 3
    for t, r, res in zip(targets, rungs, results):
        p = pycvc.simplify_params()
        p.target_tris = t
        one, one_res = pycvc.simplify(g, p)
        assert pycvc.content_hash(one) == pycvc.content_hash(r)
        assert np.array_equal(one.vertices(), r.vertices())
        assert one_res.world_error == res.world_error and res.out_tris == r.num_triangles()
    # numpy integers are ints too (an array, or np.int64 items in a list)
    for np_targets in (np.array(targets), [np.int64(t) for t in targets]):
        np_rungs, _ = pycvc.simplify_progressive(g, np_targets)
        assert [pycvc.content_hash(r) for r in np_rungs] == [pycvc.content_hash(r) for r in rungs]
    _expect(TypeError, pycvc.simplify_progressive, g, [1000, 2.5])
    _expect(OverflowError, pycvc.simplify_progressive, g, [-1])
    print("  ok: simplify_progressive snapshots == simplify at each target (list or numpy)")


# ── pyramids ─────────────────────────────────────────────────────────────────


def test_mesh_pyramid_ladder_and_pool():
    g = _terrain(64)
    pp = pycvc.pyramid_params()
    pp.max_rungs = 3
    pyr = pycvc.build_mesh_pyramid(g, pp)
    tris = pyr.rung_triangles()
    err = list(pyr.world_error_m)
    assert len(pyr) == pyr.rung_count() == 4 and tris[0] == 8192
    assert all(a > b for a, b in zip(tris, tris[1:]))
    assert err[0] == 0.0 and pycvc.ladder_is_monotonic(err) and err[-1] > 0.0
    assert len(pyr.rungs) == 4 and pyr.rungs[0].num_triangles() == 8192
    # pooled build is bit-identical (an owned pool, and the app's shared one)
    for pool in (pycvc.thread_pool(3), app.compute_pool()):
        p2 = pycvc.build_mesh_pyramid(g, params=pp, pool=pool)
        assert _hashes(p2) == _hashes(pyr) and list(p2.world_error_m) == err
    # IndexError for a rung outside the ladder
    _expect(IndexError, pyr.rung, 4)
    # a hand-built pyramid: rungs/world_error_m assign from plain sequences
    hand = pycvc.mesh_pyramid()
    hand.rungs = [pyr.rung(0), pyr.rung(2)]
    hand.world_error_m = [0.0, err[2]]
    assert hand.rung_triangles() == [tris[0], tris[2]] and tuple(hand.world_error_m) == (0.0, err[2])
    print("  ok: mesh pyramid %s tris, err %s; pool == serial" % (tris, ["%.2f" % e for e in err]))


def test_image_and_volume_pyramids():
    img = pycvc.image(64, 32, pycvc.image.RGBA, pycvc.image.u8)
    img.numpy()[:] = 200
    ipyr = pycvc.build_image_pyramid(img)
    dims = [(ipyr.rung(k).width(), ipyr.rung(k).height()) for k in range(len(ipyr))]
    assert dims[0] == (64, 32) and dims[1] == (32, 16) and len(ipyr.rungs) == len(dims)
    assert ipyr.world_error_m[0] == 0.0 and pycvc.ladder_is_monotonic(list(ipyr.world_error_m))
    assert int(ipyr.rung(1).numpy()[0, 0, 0]) == 200  # box average of a flat image
    vol = pycvc.volume(app)
    vol.set_float_grid([float(i % 7) for i in range(32 ** 3)], 32, 32, 32, 0, 0, 0, 1, 1, 1)
    vpyr = pycvc.build_volume_pyramid(vol, pool=pycvc.thread_pool(2))
    vd = [vpyr.rung(k).xdim() for k in range(len(vpyr))]
    assert vd[:3] == [32, 16, 8] and len(vpyr.world_error_m) == len(vd)
    print("  ok: image pyramid %s, volume pyramid %s" % (dims, vd))


# ── selection math ───────────────────────────────────────────────────────────


def test_selection_math():
    bal = pycvc.preset_view("balanced")
    assert bal.desired_pixel_error == pycvc.preset_view(pycvc.quality_preset_balanced).desired_pixel_error
    assert pycvc.preset_view("aggressive").desired_pixel_error > bal.desired_pixel_error
    _expect(ValueError, pycvc.preset_view, "ultra")
    k = pycvc.k_px(bal)
    assert math.isclose(k, bal.viewport_h_px / (2 * bal.tan_half_fov))
    # the crossover and its inverse
    r = pycvc.switch_radius_m(0.5, bal)
    assert math.isclose(pycvc.screen_error_px(0.5, r, bal), bal.desired_pixel_error)
    assert math.isclose(pycvc.world_error_for_switch_radius(r, bal), 0.5)
    assert pycvc.switch_radius_m(0.0, bal) == 0.0
    assert math.isclose(pycvc.screen_radius_px(1.0, 10.0, bal), k / 10.0)
    assert pycvc.impostor_switch_radius_m(1.0, 32.0, bal) > 0.0
    # rung choice: near -> finest, far -> coarsest, hysteresis holds a boundary
    ladder = [0.0, 0.1, 1.0, 10.0]
    assert pycvc.select_rung(1.0, ladder, -1, bal) == 0
    assert pycvc.select_rung(1e7, ladder, -1, bal) == 3
    r1 = pycvc.switch_radius_m(0.1, bal)
    just_past = r1 * 1.05  # inside the 15% coarsen band
    assert pycvc.select_rung(just_past, ladder, -1, bal) == 1  # no history
    assert pycvc.select_rung(just_past, ladder, 0, bal) == 0  # held by hysteresis
    assert pycvc.select_rung(r1 * 1.2, ladder, 0, bal) == 1
    assert not pycvc.ladder_is_monotonic([0.0, 2.0, 1.0])
    # bound-nearest distance, view_params.eye as a property
    v = pycvc.view_at(0.0, 0.0, 100.0)
    assert v.eye == (0.0, 0.0, 100.0)
    assert math.isclose(pycvc.bound_distance_m((0.0, 0.0, 0.0), 10.0, v), 90.0)
    assert pycvc.bound_distance_m([0.0, 0.0, 100.0], 10.0, v) == v.z_near  # inside the bound
    _expect(ValueError, pycvc.bound_distance_m, (0.0, 0.0), 1.0, v)
    v.eye = [1.0, 2.0, 3.0]
    assert v.eye == (1.0, 2.0, 3.0)
    # orthographic: distance plays no part
    o = pycvc.preset_view()
    o.ortho_px_per_m = 4.0
    assert pycvc.screen_error_px(0.25, 1e6, o) == 1.0
    assert pycvc.fade_alpha(1.0, 0.0) == 1.0 and 0.0 < pycvc.fade_alpha(0.1, 1.0) < 1.0
    print("  ok: presets, k_px, crossovers, select_rung (+hysteresis), bound distance, ortho")


# ── tiles ────────────────────────────────────────────────────────────────────


def _city_parts(nx=3, ny=3, pitch=100.0, n=12):
    parts = []
    for j in range(ny):
        for i in range(nx):
            parts.append(("ground_%d_%d" % (i, j), _terrain(n, i * pitch, j * pitch, pitch, amp=1.0)))
            b = "b%d_%d" % (i, j)
            cx, cy = i * pitch + 50.0, j * pitch + 50.0
            parts.append((b + "_walls", _box(cx, cy, 20.0, 30.0)))
            parts.append((b + "_roof", _box(cx, cy, 22.0, 2.0, z0=30.0)))
    return parts


def test_partition_parts_and_group_keys():
    parts = _city_parts()
    tiles = pycvc.partition_parts(parts, 100.0)
    assert len(tiles) == 9
    t0 = tiles[0]
    assert (t0.cell.i, t0.cell.j) == (0, 0) and t0.cell == pycvc.cell_index()
    assert set(t0.parts) == {"ground_0_0", "b0_0_walls", "b0_0_roof"}
    assert t0.geom.num_triangles() == 12 * 12 * 2 + 24
    mn, mx = t0.bounds[:3], t0.bounds[3:]
    assert mn[0] == 0.0 and mx[0] == 100.0 and mx[2] == 32.0
    # row-major cells, independent of the input order; content_hash is canonical
    rev = pycvc.partition_parts(list(reversed(parts)), 100.0)
    assert [(t.cell.i, t.cell.j) for t in rev] == [(t.cell.i, t.cell.j) for t in tiles]
    assert [t.content_hash for t in rev] == [t.content_hash for t in tiles]
    assert t0.content_hash == pycvc.content_hash(t0.geom)
    # named_part objects work too, and keep their geometry alive
    named = [pycvc.named_part(n, g) for n, g in parts]
    del parts
    assert named[0].name == "ground_0_0" and named[0].geometry.num_triangles() == 288
    assert [t.content_hash for t in pycvc.partition_parts(named, 100.0)] == [t.content_hash for t in tiles]
    # A roof whose centroid sits in the NEXT cell: alone it lands there; grouped
    # with its walls (by suffix list or by callable) it travels with them.
    split = [("x_walls", _box(95.0, 50.0, 6.0, 10.0)), ("x_roof", _box(102.0, 50.0, 8.0, 1.0, z0=10.0))]
    assert len(pycvc.partition_parts(split, 100.0)) == 2
    assert len(pycvc.partition_parts(split, 100.0, group_key=["_walls", "_roof"])) == 1
    seen = []

    def key(name):
        seen.append(name)
        return name.split("_")[0]

    assert len(pycvc.partition_parts(split, 100.0, key)) == 1 and sorted(seen) == ["x_roof", "x_walls"]
    # partition_components over a merged mesh, with a partition_params origin
    merged = pycvc.geometry(app)
    for _, g in split:
        merged.merge(g)
    # partition_model over a model's meshes (an empty model -> no tiles; a model
    # with meshes needs a file loader, which this build may not have)
    assert len(pycvc.partition_model(pycvc.model(), 100.0, group_key=["_walls"])) == 0
    comp = pycvc.partition_components(merged, 100.0)
    assert len(comp) == 2 and comp[0].parts == ("component_0",)
    pp = pycvc.partition_params()
    pp.origin = (-50.0, 0.0, 0.0)  # shift the grid: both centroids now share a cell
    assert pp.origin == (-50.0, 0.0, 0.0) and pp.up_axis == 2
    comp = pycvc.partition_components(merged, 100.0, pp)
    assert len(comp) == 1 and comp[0].parts == ("component_0", "component_1")
    print("  ok: partition_parts / partition_components, suffix + callable group keys")


def test_tile_geom_and_cell_outlive_the_tile():
    """tile.geom / tile.cell are owned copies, not pointers into the tile: they
    stay valid after the tile is gone -- a temporary from indexing a result, or
    a tile the on_tile callback was handed -- even once its memory is reused."""
    parts = _city_parts(nx=2, ny=1)
    tile_tris = 12 * 12 * 2 + 24
    g = pycvc.partition_parts(parts, 100.0)[1].geom  # that tile is freed here
    c = pycvc.partition_parts(parts, 100.0)[1].cell
    kept = []
    tiles = pycvc.partition_parts(parts, 100.0)
    pycvc.build_tiled_pyramids(tiles, on_tile=lambda i, t, p: kept.append((t.geom, t.cell)))
    del tiles
    gc.collect()
    # churn the allocator with other tiles, so freed tile storage is reused
    churn = [pycvc.partition_parts(_city_parts(nx=2, ny=1, n=3 + k), 100.0) for k in range(8)]
    assert g.num_triangles() == tile_tris and g.vertices().min(axis=0)[0] == 100.0
    assert (c.i, c.j) == (1, 0)
    assert sorted((kc.i, kc.j) for _, kc in kept) == [(0, 0), (1, 0)]
    assert [kg.num_triangles() for kg, _ in kept] == [tile_tris, tile_tris]
    # a copy: editing it leaves the tile alone until it is assigned back
    t = churn[0][0]
    cell = t.cell
    cell.i = 5
    assert t.cell.i == 0
    t.cell = cell
    t.geom = _box(0.0, 0.0, 1.0, 1.0)
    assert (t.cell.i, t.cell.j) == (5, 0) and t.geom.num_triangles() == 12
    print("  ok: tile.geom / tile.cell are owned copies that outlive their tile")


def test_tiled_pyramids_callback_from_pool_threads():
    tiles = pycvc.partition_parts(_city_parts(n=24), 100.0)
    pp = pycvc.pyramid_params()
    pp.max_rungs = 2
    serial = pycvc.build_tiled_pyramids(tiles, pp)
    calls = []
    main = threading.get_ident()

    def on_tile(index, tile, pyr):
        assert isinstance(tile, pycvc.tile) and isinstance(pyr, pycvc.mesh_pyramid)
        calls.append((index, threading.get_ident(), (tile.cell.i, tile.cell.j), len(pyr)))

    pooled = pycvc.build_tiled_pyramids(tiles, pp, pool=pycvc.thread_pool(3), on_tile=on_tile)
    assert sorted(c[0] for c in calls) == list(range(len(tiles)))
    for idx, _, cell, n in calls:
        assert cell == (tiles[idx].cell.i, tiles[idx].cell.j) and n == len(pooled[idx])
    assert [_hashes(p) for p in pooled] == [_hashes(p) for p in serial]
    threads = {c[1] for c in calls}
    print("  ok: build_tiled_pyramids on_tile from %d thread(s) (main among them: %s)"
          % (len(threads), main in threads))


def test_callback_exceptions_propagate():
    tiles = pycvc.partition_parts(_city_parts(), 100.0)
    pool = pycvc.thread_pool(3)

    class Boom(Exception):
        pass

    def bad_tile(index, tile, pyr):
        if index == 4:
            raise Boom("tile %d" % index)

    e = _expect(Boom, pycvc.build_tiled_pyramids, tiles, pool=pool, on_tile=bad_tile)
    assert str(e) == "tile 4"
    _expect(Boom, pycvc.build_tiled_pyramids, tiles, on_tile=bad_tile)  # serial path too
    # the pool survives a failed build
    assert len(pycvc.build_tiled_pyramids(tiles, pool=pool)) == len(tiles)

    def bad_key(name):
        raise KeyError(name)

    _expect(KeyError, pycvc.partition_parts, [("a", _box(0, 0, 1, 1))], 10.0, bad_key)
    _expect(TypeError, pycvc.partition_parts, [("a", _box(0, 0, 1, 1))], 10.0, lambda n: 7)
    _expect(TypeError, pycvc.build_tiled_pyramids, tiles, on_tile=42)
    _expect(TypeError, pycvc.partition_parts, [("a", "not a geometry")], 10.0)
    _expect(ValueError, pycvc.partition_parts, [("a", _box(0, 0, 1, 1))], 0.0)
    _expect(ValueError, pycvc.content_hash, _box(0, 0, 1, 1), -1.0)
    print("  ok: callback exceptions re-raised with their type; bad args -> TypeError/ValueError")


# ── store ────────────────────────────────────────────────────────────────────


def _baked_city():
    tiles = pycvc.partition_parts(_city_parts(n=16), 100.0)
    pyrs = pycvc.build_tiled_pyramids(tiles, pool=pycvc.thread_pool(2))
    w = pycvc.scene_writer(app)
    for i, (t, p) in enumerate(zip(tiles, pyrs)):
        w.write_mesh_pyramid("tile%d" % i, p, "%016x" % t.content_hash)
    return tiles, pyrs, w


def test_store_round_trip_through_bytes():
    if not pycvc.HAVE_LOD_STORE:
        print("  skip: libcvc built without HDF5 (no scene.cvch5 store)")
        return
    tiles, pyrs, w = _baked_city()
    img = pycvc.image(16, 16, pycvc.image.RGB, pycvc.image.u8)
    img.numpy()[:] = 77
    ipyr = pycvc.build_image_pyramid(img)
    w.write_image_pyramid("albedo", ipyr, pycvc.image_content_hash(img))
    blob = w.to_blob()
    assert isinstance(blob, bytes) and blob[:8] == b"\x89HDF\r\n\x1a\n"
    sha = hashlib.sha256(blob).hexdigest()
    readers = [pycvc.scene_reader(app, blob),                     # trusted bytes
               pycvc.scene_reader(app, bytearray(blob)),          # any bytes-like
               pycvc.scene_reader(app, memoryview(blob)),
               pycvc.scene_reader.open_verified(app, blob, sha),  # authenticated
               pycvc.scene_reader.open_verified(app, np.frombuffer(blob, np.uint8), sha.upper())]
    for r in readers:
        idx = {e.name: e for e in r.index()}
        assert len(idx) == len(tiles) + 1 and idx["albedo"].kind == "I" and idx["tile0"].kind == "M"
        for i, (t, p) in enumerate(zip(tiles, pyrs)):
            name = "tile%d" % i
            assert r.has(name) and r.has(name, "%016x" % t.content_hash) and not r.has(name, "stale")
            assert _same_ladder(r.read_mesh_pyramid(name), p)
            assert idx[name].nrungs == len(p) and tuple(idx[name].world_error_m) == tuple(p.world_error_m)
        ib = r.read_image_pyramid("albedo")
        assert len(ib) == len(ipyr) and int(ib.rung(0).numpy()[3, 3, 1]) == 77
    print("  ok: %d-byte blob -> 5 readers (bytes/bytearray/memoryview/verified/numpy)" % len(blob))


def test_store_file_backed_and_bake():
    if not pycvc.HAVE_LOD_STORE:
        print("  skip: libcvc built without HDF5 (no scene.cvch5 store)")
        return
    g = _terrain(24)
    pyr = pycvc.build_mesh_pyramid(g)
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "scene.cvch5")
        fw = pycvc.scene_writer(app, path)
        fw.write_mesh_pyramid("ground", pyr, pycvc.mesh_content_hash(g))
        blob = fw.to_blob()
        del fw
        with open(path, "rb") as f:
            assert f.read() == blob  # the blob IS the file image
        fr = pycvc.scene_reader(app, path)
        assert _same_ladder(fr.read_mesh_pyramid("ground"), pyr)
        del fr
        assert pycvc.has_pyramid(app, path, "ground", pycvc.mesh_content_hash(g))
        assert [e.name for e in pycvc.read_lod_index(app, path)] == ["ground"]
        assert _same_ladder(pycvc.read_mesh_pyramid(app, path, "ground"), pyr)
        # bake: builds, then skips the current pyramid, then rebuilds when forced
        bpath = os.path.join(d, "bake.cvch5")
        assert pycvc.bake_mesh_asset(app, bpath, "g", g) is True
        assert pycvc.bake_mesh_asset(app, bpath, "g", g) is False
        assert pycvc.bake_mesh_asset(app, bpath, "g", g, force=True, pool=pycvc.thread_pool(2)) is True
        pycvc.write_mesh_pyramid(app, bpath, "h", pyr)
        assert sorted(e.name for e in pycvc.read_lod_index(app, bpath)) == ["g", "h"]
    print("  ok: file writer/reader, to_blob == file bytes, free functions, bake skip/force")


def test_store_exceptions():
    if not pycvc.HAVE_LOD_STORE:
        print("  skip: libcvc built without HDF5 (no scene.cvch5 store)")
        return
    _, _, w = _baked_city()
    blob = w.to_blob()
    # SHA-256 mismatch: refused before HDF5 sees a byte
    e = _expect(RuntimeError, pycvc.scene_reader.open_verified, app, blob, "0" * 64)
    assert "SHA-256" in str(e)
    # a flipped byte no longer matches the digest of the original
    bad = bytearray(blob)
    bad[len(bad) // 2] ^= 0xFF
    _expect(RuntimeError, pycvc.scene_reader.open_verified, app, bad, hashlib.sha256(blob).hexdigest())
    # HDF5 / store failures are OSError
    _expect(OSError, pycvc.scene_reader, app, b"definitely not an HDF5 container")
    _expect(OSError, pycvc.scene_reader, app, blob[: len(blob) // 3])
    _expect(OSError, pycvc.scene_reader, app, "/nonexistent/dir/scene.cvch5")
    r = pycvc.scene_reader(app, blob)
    _expect(OSError, r.read_mesh_pyramid, "no-such-asset")
    _expect(OSError, r.read_image_pyramid, "tile0")  # a mesh, not an image
    # a blob must be bytes-like
    _expect(TypeError, pycvc.scene_reader.open_verified, app, 12345, "0" * 64)
    print("  ok: RuntimeError (hash), OSError (HDF5/store), TypeError (not bytes)")


# ── the GIL ──────────────────────────────────────────────────────────────────


def _assert_releases_gil(label, fn):
    """Run fn -- exactly ONE C++ call, its inputs built beforehand -- on a
    worker thread while this thread ticks in pure Python. If the call held the
    GIL, this thread could not tick for its whole duration: the largest gap
    between ticks inside the call window would be ~the window itself. Released,
    the ticks keep coming. (Several calls, or Python work, in one window would
    let this thread tick between them and hide a call that held the GIL.)"""
    window = {}

    def run():
        window["t0"] = time.perf_counter()
        window["result"] = fn()
        window["t1"] = time.perf_counter()

    th = threading.Thread(target=run)
    ticks = []
    th.start()
    while th.is_alive():
        ticks.append(time.perf_counter())
        sum(range(50))  # a little pure-Python work per tick
    th.join()
    t0, t1 = window["t0"], window["t1"]
    inside = [t for t in ticks if t0 <= t <= t1]
    span = t1 - t0
    gaps = [b - a for a, b in zip([t0] + inside, inside + [t1])]
    worst = max(gaps)
    assert span > 0.02, "%s finished too fast (%.3fs) to judge" % (label, span)
    assert len(inside) > 20 and worst < 0.5 * span, (
        "%s held the GIL: %d ticks, worst gap %.3fs of a %.3fs call" % (label, len(inside), worst, span))
    print("  ok: %-24s %.3fs call, %6d main-thread ticks, worst gap %.4fs"
          % (label, span, len(inside), worst))
    return window["result"]


def test_long_calls_release_the_gil():
    big = _terrain(128)  # 32768 triangles
    _assert_releases_gil("build_mesh_pyramid", lambda: pycvc.build_mesh_pyramid(big))
    _assert_releases_gil("simplify", lambda: pycvc.simplify(big))
    tiles = pycvc.partition_parts([("t%d" % i, _terrain(64, 100.0 * i, 0.0)) for i in range(6)], 100.0)
    pool = pycvc.thread_pool(2)
    _assert_releases_gil("build_tiled_pyramids",
                         lambda: pycvc.build_tiled_pyramids(tiles, pool=pool, on_tile=lambda i, t, p: None))
    _assert_releases_gil("simplify_progressive", lambda: pycvc.simplify_progressive(big, [8000, 2000, 500]))
    parts = [("t%d" % i, _terrain(32, 25.0 * (i % 8), 25.0 * (i // 8), 25.0)) for i in range(64)]
    _assert_releases_gil("partition_parts", lambda: pycvc.partition_parts(parts, 50.0))
    dense = _terrain(320)  # 204800 triangles
    _assert_releases_gil("content_hash", lambda: pycvc.content_hash(dense))
    img = pycvc.image(1536, 1024, pycvc.image.RGBA, pycvc.image.u8)
    _assert_releases_gil("build_image_pyramid", lambda: pycvc.build_image_pyramid(img))


def test_store_calls_release_the_gil():
    if not pycvc.HAVE_LOD_STORE:
        print("  skip: libcvc built without HDF5 (no scene.cvch5 store)")
        return
    # Three full-size rungs of a 524288-triangle mesh (the store does not care
    # how a ladder was made): a ~57 MB container, so each call below runs for
    # 50 ms or more.
    big = _terrain(512)
    pyr = pycvc.mesh_pyramid()
    pyr.rungs = [big, big, big]
    pyr.world_error_m = [0.0, 1.0, 2.0]
    w = pycvc.scene_writer(app)
    _assert_releases_gil("write_mesh_pyramid", lambda: w.write_mesh_pyramid("p", pyr))
    blob = _assert_releases_gil("to_blob", w.to_blob)
    r = _assert_releases_gil("scene_reader(bytes)", lambda: pycvc.scene_reader(app, blob))
    sha = hashlib.sha256(blob).hexdigest()
    _assert_releases_gil("open_verified", lambda: pycvc.scene_reader.open_verified(app, blob, sha))
    back = _assert_releases_gil("read_mesh_pyramid", lambda: r.read_mesh_pyramid("p"))
    assert back.rung_triangles() == [524288] * 3
    src = _terrain(80)
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "bake.cvch5")
        assert _assert_releases_gil("bake_mesh_asset", lambda: pycvc.bake_mesh_asset(app, path, "g", src))


def test_thread_pool_handles():
    p = pycvc.thread_pool(3)
    assert p.concurrency() == 4
    assert pycvc.thread_pool(1).concurrency() == 2  # one worker + the caller
    cp = app.compute_pool()
    assert cp.concurrency() >= 1 and cp._pycvc_app is app
    print("  ok: thread_pool(n).concurrency() == n + 1; app.compute_pool()")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            print(name)
            fn()
    print("pycvc LOD tests: OK")
