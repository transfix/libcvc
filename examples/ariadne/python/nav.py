"""nav.py — a Python loop steps a cvc::nav sim_world (pycvc) and draws it in nav.ari.

    python nav.py                           # a window
    python nav.py --offscreen --png nav.png --frames 240
    python nav.py --headless --frames 200   # the sim and the readouts, nothing drawn

The agents shuttle across an 80 m world through a field of pillars they discover as they sense
(each starts knowing none): one that reaches its goal is sent back to the other side. Every frame the
script steps the world `sim.ticks` times, redraws the agents (a triangle each, pointing along its
heading), their tracks and the pillars in the document's scene, publishes the readouts and agent
0's belief, and reads the Sim window's controls. Restart and New goals are document events this
script handles.
"""

import math
import os
import sys

import numpy as np
import pycvc
import pycvc_gl

from pycvc_gl.ariadne import Host, main_args

HERE = os.path.dirname(os.path.abspath(__file__))
BOUNDS = (-40.0, -40.0, 40.0, 40.0)  # world metres: 81 x 81 one-metre cells
CELLS = 81
SCALE = 0.05  # the world's normalized units: world metres * SCALE
SIDE_X = 34.0  # the goals are at x = +-SIDE_X
# Square pillars, 4 m across, by their south-west corners (m). Convex obstacles: nav's default policy
# is a potential field, which a concave wall can trap.
PILLARS = [(-22, -20), (-22, 4), (-12, -8), (-12, 18), (-2, -26), (-2, 4), (8, -12), (8, 20),
           (18, -2), (18, -28), (26, 14)]
WALLS = [(x, y, x + 3, y + 3) for x, y in PILLARS]  # x0 y0 x1 y1, cells included


def walls_grid():
    """The true occupancy, (rows, cols) uint8, row 0 at min y."""
    occ = np.zeros((CELLS, CELLS), np.uint8)
    for x0, y0, x1, y1 in WALLS:
        occ[y0 + 40:y1 + 41, x0 + 40:x1 + 41] = 1
    return occ


def make_world(n, rng):
    """n agents in the west, their goals in the east; nav's default policy and kinematic bicycle."""
    start = np.stack([np.full(n, -SIDE_X), np.linspace(-30.0, 30.0, n)], axis=1)
    goal = np.stack([np.full(n, SIDE_X), rng.uniform(-30.0, 30.0, n)], axis=1)
    occ = walls_grid()
    color = np.ones((n, 3), np.float32)
    # Lengths and speeds in the world's normalized units (metres * SCALE): radius 1.2 m, barrier
    # range 3 m, top speed 8 m/s, wheelbase 3.3 m; a 15 m sensor of 120 rays all round, sensing
    # every 4th tick; the prior is empty and unknown cells count as free (optimistic).
    return pycvc.nav_sim_world_create(
        occ, np.zeros_like(occ), "", (start * SCALE).astype(np.float32),
        (goal * SCALE).astype(np.float32), color, CELLS, CELLS, *BOUNDS, 0.0, 0.0, SCALE,
        15.0, 120, 2 * math.pi,  # sensor range (m), rays, field of view
        0.06, 0.15, 0.06, 0.4, 0.165, 0.6, 0.12, 0.12,  # rr d_hat dt vmax L delta a a_lat
        0.8, 1, 1, 0.2,  # k_steer, substeps, reverse allowed, reach tolerance
        4, 0, 2.2, -1.4, 8.0, 1, 0.5, 0.15, 4.0), goal  # sensing, belief log-odds


def box_mesh(boxes, height=2.5):
    """Boxes (x0, y0, x1, y1, cells included) as one mesh: flat (vertices, triangles)."""
    verts, tris = [], []
    for x0, y0, x1, y1 in boxes:
        b = len(verts) // 3
        x1, y1 = x1 + 1, y1 + 1  # a box covers its last cell too
        for z in (0.0, height):
            verts += [float(v) for v in (x0, y0, z, x1, y0, z, x1, y1, z, x0, y1, z)]
        for f in ((0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6), (0, 4, 5), (0, 5, 1), (1, 5, 6),
                  (1, 6, 2), (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0)):
            tris += [b + f[0], b + f[1], b + f[2]]
    return verts, tris


def agents_geometry(app, pos, heading, z=0.6, length=2.2, width=1.1):
    """One triangle per agent, its nose along its heading."""
    g = pycvc.geometry(app)
    verts, tris = [], []
    for i, ((x, y), h) in enumerate(zip(pos.tolist(), heading.tolist())):  # Python floats
        c, s = math.cos(h), math.sin(h)
        verts += [x + c * length, y + s * length, z,
                  x - c * length * 0.5 - s * width, y - s * length * 0.5 + c * width, z,
                  x - c * length * 0.5 + s * width, y - s * length * 0.5 - c * width, z]
        tris += [3 * i, 3 * i + 1, 3 * i + 2]
    g.add_vertices(verts)
    g.add_triangles(tris)
    return g


def tracks_geometry(app, tracks, z=0.3):
    """Every agent's track as line segments."""
    g = pycvc.geometry(app)
    verts, lines = [], []
    for t in tracks:
        if len(t) < 2:
            continue
        b = len(verts) // 3
        for x, y in t:
            verts += [x, y, z]
        for k in range(len(t) - 1):
            lines += [b + k, b + k + 1]
    if verts:
        g.add_vertices(verts)
        g.add_lines(lines)
    return g


def main(argv=None):
    args = main_args(argv, frames=0)
    host = Host(mode=args.mode, name="nav", width=args.width, height=args.height)
    rng = np.random.default_rng(3)
    sim = {}

    def restart():
        n = max(1, host.get_int("sim.agents", 12))
        sim["world"], sim["goal"] = make_world(n, rng)
        sim["n"] = n
        sim["tracks"] = [[] for _ in range(n)]
        sim["speeds"] = []
        sim["tick"] = 0
        sim["trips"] = 0

    def retarget(i, x):
        gy = rng.uniform(-30.0, 30.0)
        sim["goal"][i] = (x, gy)
        pycvc.nav_sim_world_retarget(sim["world"], i, x * SCALE, gy * SCALE)

    @host.on("restart")
    def on_restart():
        restart()

    @host.on("shuffle")
    def on_shuffle():  # a new goal for everyone, on the side each is heading for
        for i in range(sim["n"]):
            retarget(i, sim["goal"][i][0])

    host.load(os.path.join(HERE, "nav.ari"))  # its warnings are logged
    restart()

    drawn = host.mode != "headless"
    if drawn:
        lab = host.lab  # a pycvc_gl Lab over the document's scene
        verts, tris = box_mesh(WALLS)
        lab.add_mesh("pillars", verts, tris, color=(0.45, 0.47, 0.52))
        host.scene.addGraphics("agents", pycvc.geometry(host.app))
        host.scene.addGraphics("tracks", pycvc.geometry(host.app))
        host.frame_bounds((-42, -42, 0), (42, 42, 4))  # z is up

    @host.on_frame
    def tick(dt):
        if host.get_int("sim.agents", sim["n"]) != sim["n"]:
            restart()  # the Agents slider moved
        if not host.get_bool("sim.paused"):
            for _ in range(max(1, host.get_int("sim.ticks", 2))):
                pycvc.nav_sim_world_step(sim["world"])
                sim["tick"] += 1
        pos, heading, speed, _mode, reached = pycvc.nav_sim_world_snapshot(sim["world"])
        if sim["tick"] % 4 == 0:
            for i, p in enumerate(pos):  # the last 150 points of each track
                sim["tracks"][i] = sim["tracks"][i][-149:] + [(float(p[0]), float(p[1]))]
        for i in np.flatnonzero(reached):  # arrived: back to the other side
            sim["trips"] += 1
            retarget(int(i), -sim["goal"][i][0])
        sim["speeds"] = (sim["speeds"] + [float(np.mean(np.abs(speed)))])[-200:]
        host.set_many({"tick": sim["tick"], "trips": sim["trips"],
                       "speed": f"{np.mean(np.abs(speed)):.2f}"}, prefix="stats")
        host.set_series("speeds", sim["speeds"])
        if host.frame_count % 10 == 0:  # what agent 0 has seen: its belief plane (all share one)
            seen = pycvc.nav_sim_world_belief_occ(sim["world"], 0)[::-1]  # row 0 at the top
            img = np.where(seen[..., None] > 0, [230, 120, 60], [30, 34, 40]).astype(np.uint8)
            host.publish_image("belief", np.kron(img, np.ones((2, 2, 1), np.uint8)))
        if drawn:  # a new mesh resets a node's material, so it is styled after each one
            agents = host.scene.geometry_node("agents")
            agents.setGeometry(agents_geometry(host.app, pos, heading))
            agents.setUseSingleColor(True)
            agents.setColor(0.35, 0.85, 1.0)
            tracks = host.scene.geometry_node("tracks")
            tracks.setGeometry(tracks_geometry(host.app, sim["tracks"]
                                               if host.get_bool("show.tracks") else []))
            tracks.setRenderMode(pycvc_gl.GeometryRenderMode_LINES)  # segments, not triangles
            tracks.setLineWidth(2.0)
            tracks.setUseSingleColor(True)
            tracks.setColor(1.0, 0.85, 0.3)

    host.run(fps=30, frames=args.frames or None)
    if args.png:
        host.snapshot(args.png)
    print("tick:", host.get("stats.tick"), "trips:", host.get("stats.trips"))
    code = host.exit_code()
    host.close()
    return code


if __name__ == "__main__":
    sys.exit(main())
