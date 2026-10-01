#!/usr/bin/env python3
"""lod_city -- a synthetic tiled city through the whole cvc::lod pipeline, from Python.

    BUILD      a G x G block city: per block a terrain patch with rolling relief
               and four buildings, each a tessellated "<b>_walls" part plus a
               "<b>_roof" part (synthetic, procedural -- no real-world data);
    PARTITION  the parts into ground tiles (partition_parts; the suffix group
               key keeps every roof with its walls);
    ATTACH     one LodGraphicsNode per tile with setBase(tile.geom): the city
               draws at full detail at once;
    BUILD      every tile's LOD pyramid on a LOADER thread with a thread_pool of
               its own. The build releases the GIL, so the frame loop here keeps
               running; on_tile queues each finished pyramid and the loop hands it
               to appendRungs (rung 0's node -- and any GPU buffers -- are kept);
    BAKE       the pyramids into an in-memory scene.cvch5 -> bytes, plus its
               SHA-256 (the digest a signed manifest would carry);
    RELOAD     the bytes with scene_reader.open_verified into a fresh scene and
               attach every tile with setPyramid;
    FLY        a camera path from street level out to a distant overview,
               printing SceneGraph.selectLOD's lod_stats at every pose.

Headless by default: rung selection is pure math on a pycvc.view_params, so no
GL context is needed. --render additionally draws every pose offscreen, takes
the view from the live camera (SceneRenderer.make_view_params) and writes one
PNG per pose, with each rung tinted (green = finest ... red = coarsest).

    python3 lod_city.py [--blocks 6] [--terrain-n 32] [--workers 3]
                        [--preset balanced] [--render OUTDIR]

run() returns the measurements as a dict; test_pycvc_gl_lod.py asserts on it.
"""

import argparse
import hashlib
import os
import queue
import sys
import threading
import time

import numpy as np

import pycvc
import pycvc_gl

PITCH = 100.0  # metres per block == tile edge
RUNG_COLORS = [(0.35, 0.75, 0.35), (0.85, 0.80, 0.30), (0.90, 0.55, 0.25), (0.85, 0.30, 0.25),
               (0.60, 0.25, 0.60)]


# ── the synthetic city ───────────────────────────────────────────────────────


def ground_height(x, y, amp):
    """Rolling relief: a few metres of hills and hummocks. Its detail is what
    decimation trades away, so it sets the ladder: on 32-cell, 100 m tiles at
    amp = 2 the measured rung errors come out near 0.2 / 0.7 / 2.3 m, i.e.
    switch radii of roughly 110 / 325 / 1100 m at the balanced preset -- the
    finest rung around the camera, the coarsest beyond a kilometre."""
    return amp * (np.sin(0.0525 * x) * np.cos(0.0425 * y) + 0.5 * np.sin(0.1325 * (x + y)) +
                  0.25 * np.cos(0.275 * x - 0.175 * y))


def _grid_mesh(app, P, nu, nv):
    """Triangulate a (nv+1) x (nu+1) x 3 lattice of points."""
    g = pycvc.geometry(app)
    g.add_vertices(P.reshape(-1).tolist())
    i, j = np.meshgrid(np.arange(nu), np.arange(nv))
    a = (j * (nu + 1) + i).ravel()
    b, c = a + 1, a + nu + 1
    d = c + 1
    g.add_triangles(np.stack([a, b, d, a, d, c], axis=1).ravel().tolist())
    return g


def terrain_patch(app, x0, y0, n, amp):
    xs = np.linspace(x0, x0 + PITCH, n + 1)
    ys = np.linspace(y0, y0 + PITCH, n + 1)
    X, Y = np.meshgrid(xs, ys)
    return _grid_mesh(app, np.stack([X, Y, ground_height(X, Y, amp)], axis=-1), n, n)


def building(app, cx, cy, w, h, z0, nu=4, nv=8):
    """A box building as two parts: walls (four nu x nv facade grids) and a flat
    roof grid. Neighbouring faces share their edge points bit for bit, so the
    simplifier welds them into one surface; the facades are planar, so most of
    their triangles go for (almost) no error."""
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - w / 2, cy + w / 2
    xs, ys = np.linspace(x0, x1, nu + 1), np.linspace(y0, y1, nu + 1)
    zs = np.linspace(z0, z0 + h, nv + 1)
    walls = pycvc.geometry(app)
    # Counter-clockwise seen from above, every face built from the SAME
    # coordinate arrays as its neighbours and the roof.
    for X, Y in ((xs, np.full_like(xs, y0)), (np.full_like(ys, x1), ys),
                 (xs[::-1], np.full_like(xs, y1)), (np.full_like(ys, x0), ys[::-1])):
        P = np.stack(np.broadcast_arrays(X[None, :], Y[None, :], zs[:, None]), axis=-1)
        walls.merge(_grid_mesh(app, P, nu, nv))
    X, Y = np.meshgrid(xs, ys)
    roof = _grid_mesh(app, np.stack([X, Y, np.full_like(X, z0 + h)], axis=-1), nu, nu)
    return walls, roof


def make_city(app, blocks=6, terrain_n=32, amp=2.0, seed=7):
    """(name, geometry) parts of a blocks x blocks city, PITCH metres a block."""
    rng = np.random.RandomState(seed)
    parts = []
    for j in range(blocks):
        for i in range(blocks):
            x0, y0 = i * PITCH, j * PITCH
            parts.append(("ground_%d_%d" % (i, j), terrain_patch(app, x0, y0, terrain_n, amp)))
            for q, (fx, fy) in enumerate(((0.28, 0.28), (0.72, 0.28), (0.28, 0.72), (0.72, 0.72))):
                cx, cy = x0 + fx * PITCH, y0 + fy * PITCH
                w = rng.uniform(14.0, 26.0)
                h = rng.uniform(8.0, 60.0)
                z0 = float(ground_height(cx, cy, amp)) - 1.0  # sunk a metre into the ground
                walls, roof = building(app, cx, cy, w, h, z0)
                name = "b%d_%d_%d" % (i, j, q)
                parts.append((name + "_walls", walls))
                parts.append((name + "_roof", roof))
    return parts


def camera_path(blocks):
    """(label, eye, focal) poses: street level in the middle of town, out to a
    distant overview. Distances scale with the city's size."""
    c = blocks * PITCH / 2.0
    span = blocks * PITCH
    return [
        ("street", (c, c, 2.0), (c + 50.0, c + 200.0, 2.0)),
        ("rooftops", (c, c, 90.0), (c + 100.0, c + 150.0, 0.0)),
        ("city edge", (c, -0.3 * span, 120.0), (c, c, 0.0)),
        ("1.5 spans out", (c, -1.5 * span, 0.6 * span), (c, c, 0.0)),
        ("5 spans out", (c, -5.0 * span, 2.0 * span), (c, c, 0.0)),
        ("12 spans out", (c, -12.0 * span, 5.0 * span), (c, c, 0.0)),
    ]


def _stats_dict(st):
    return {"nodes": st.nodes, "hidden": st.hidden, "changes": st.changes,
            "rung_nodes": list(st.rung_nodes), "drawn_tris": st.drawn_tris,
            "full_tris": st.full_tris}


# ── the pipeline ─────────────────────────────────────────────────────────────


def run(blocks=6, terrain_n=32, workers=3, preset="balanced", render_dir=None, verbose=True,
        frame_dt=1.0 / 240.0):
    say = print if verbose else (lambda *a, **k: None)
    app = pycvc.make_app()

    # BUILD + PARTITION
    t0 = time.perf_counter()
    parts = make_city(app, blocks, terrain_n)
    tiles = pycvc.partition_parts(parts, PITCH, group_key=["_walls", "_roof"])
    src_tris = sum(t.geom.num_triangles() for t in tiles)
    say("city: %d parts -> %d tiles, %d triangles (%.2fs)"
        % (len(parts), len(tiles), src_tris, time.perf_counter() - t0))

    # ATTACH at full detail at once
    style = None
    if render_dir:
        def style(node):  # rung k's tint, from the rung node's name "lod<k>"
            k = int(node.getName()[3:])
            node.setColor(*RUNG_COLORS[min(k, len(RUNG_COLORS) - 1)])
    sg = pycvc_gl.SceneGraph(app, "lodcity")
    sg.add_group("city")
    nodes = []
    for k, t in enumerate(tiles):
        n = sg.add_child_lod("city", "tile%02d" % k)
        if style:
            n.setRungStyle(style)
        n.setBase(t.geom)
        nodes.append(n)

    # BUILD PYRAMIDS on a loader thread; the frame loop keeps going meanwhile.
    pp = pycvc.pyramid_params()
    pp.max_rungs = 3
    finished = queue.Queue()
    loader = {}

    def load():
        try:
            pool = pycvc.thread_pool(workers)  # the loader's OWN pool (see tiles.h)
            t = time.perf_counter()
            loader["pyramids"] = pycvc.build_tiled_pyramids(
                tiles, pp, pool=pool, on_tile=lambda i, tile, pyr: finished.put((i, pyr)))
            loader["seconds"] = time.perf_counter() - t
        except BaseException as e:  # surfaced on the main thread below
            loader["error"] = e

    street = pycvc.view_at(*camera_path(blocks)[0][1], preset=preset)
    th = threading.Thread(target=load, name="lod-loader")
    frames = kept = 0
    th.start()
    while th.is_alive() or not finished.empty():
        try:
            while True:  # drain what finished since the last frame
                i, pyr = finished.get_nowait()
                kept += bool(nodes[i].appendRungs(pyr))
        except queue.Empty:
            pass
        sg.selectLOD(street)  # the per-frame pass, on the scene's owner thread
        frames += 1
        time.sleep(frame_dt)  # stand-in for the rest of a frame
    th.join()
    if "error" in loader:
        raise loader["error"]
    pyramids = loader["pyramids"]
    rungs = sorted({len(p) for p in pyramids})
    say("pyramids: %d tiles on %d workers in %.2fs; %d frames ran meanwhile; "
        "appendRungs kept rung 0 on %d/%d tiles; rungs per tile %s"
        % (len(pyramids), workers + 1, loader["seconds"], frames, kept, len(nodes), rungs))

    # BAKE -> bytes + digest
    t = time.perf_counter()
    writer = pycvc.scene_writer(app)
    for k, (tile, pyr) in enumerate(zip(tiles, pyramids)):
        writer.write_mesh_pyramid("tile%02d" % k, pyr, "%016x" % tile.content_hash)
    blob = writer.to_blob()
    sha = hashlib.sha256(blob).hexdigest()
    say("bake: %d-byte scene.cvch5 blob, sha256 %s... (%.2fs)"
        % (len(blob), sha[:16], time.perf_counter() - t))

    # RELOAD, authenticated, into a fresh scene
    reader = pycvc.scene_reader.open_verified(app, blob, sha)
    scene = pycvc_gl.SceneGraph(app, "lodcity_reloaded")
    scene.add_group("city")
    reloaded = []
    for entry in reader.index():
        n = scene.add_child_lod("city", entry.name)
        if style:
            n.setRungStyle(style)
        n.setPyramid(reader.read_mesh_pyramid(entry.name))
        reloaded.append(n)
    assert [n.rung_triangle_counts() for n in reloaded] == [p.rung_triangles() for p in pyramids]
    say("reload: open_verified -> %d LodGraphicsNodes, ladders identical to the build"
        % len(reloaded))

    # FLY
    view = None
    if render_dir:
        os.makedirs(render_dir, exist_ok=True)
        view = pycvc_gl.SceneRenderer(scene, 960, 540, True, "lodcity")
        view.setBackground(0.62, 0.72, 0.85)
        scene.setDiagnosticChromeVisible(False)  # no grid / bbox labels in the shots
    base = pycvc.preset_view(preset)
    poses = []
    say("\n%-14s %8s %7s  %-18s %10s %10s %7s" % ("pose", "dist", "changes", "nodes per rung",
                                                 "drawn", "full", "saving"))
    for label, eye, focal in camera_path(blocks):
        if view is not None:
            view.setCamera(*eye, *focal, 0.0, 0.0, 1.0, 45.0, 0.5, 1e5)
            v = view.make_view_params(base)  # eye / viewport / fov from the live camera
        else:
            v = pycvc.view_at(*eye, preset=preset)
        st = scene.select_lod_stats(v)
        if view is not None:
            view.writePNG(os.path.join(render_dir, "lod_city_%02d.png" % len(poses)))
        c = blocks * PITCH / 2.0
        dist = float(np.linalg.norm(np.subtract(eye, (c, c, 0.0))))
        say("%-14s %7.0fm %7d  %-18s %10d %10d %6.1fx"
            % (label, dist, st.changes, st.rung_nodes, st.drawn_tris, st.full_tris, st.saving()))
        poses.append({"label": label, "eye": eye, "distance_m": dist, **_stats_dict(st)})

    # the A/B switch: LOD off pins rung 0 everywhere
    scene.setLODEnabled(False)
    off = scene.select_lod_stats(pycvc.view_at(*camera_path(blocks)[-1][1], preset=preset))
    scene.setLODEnabled(True)
    say("LOD off at the last pose: drawn %d of %d" % (off.drawn_tris, off.full_tris))
    if view is not None:
        view.close()
        say("rendered %d poses to %s" % (len(poses), render_dir))

    return {"tiles": len(tiles), "source_tris": src_tris, "rungs_per_tile": rungs,
            "build_seconds": loader["seconds"], "frames_during_build": frames,
            "rung0_kept": kept, "blob_bytes": len(blob), "sha256": sha,
            "poses": poses, "lod_off": _stats_dict(off)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--blocks", type=int, default=6, help="city is BLOCKS x BLOCKS tiles")
    ap.add_argument("--terrain-n", type=int, default=32, help="terrain grid cells per tile edge")
    ap.add_argument("--workers", type=int, default=3, help="loader pool workers")
    ap.add_argument("--preset", default="balanced", choices=sorted(pycvc.QUALITY_PRESETS))
    ap.add_argument("--render", metavar="OUTDIR", help="also render each pose offscreen to PNGs")
    args = ap.parse_args(argv)
    r = run(args.blocks, args.terrain_n, args.workers, args.preset, args.render)
    near, far = r["poses"][0], r["poses"][-1]
    print("\nnear -> far: %d -> %d drawn triangles (%.1fx fewer)"
          % (near["drawn_tris"], far["drawn_tris"], near["drawn_tris"] / max(1, far["drawn_tris"])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
