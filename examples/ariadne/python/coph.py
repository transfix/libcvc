"""coph.py — run coph.ari: CoPH's crossing demo, on pycvc's NavCophFleet (libcvc's C++ CoPH, no torch).

    python coph.py                                 # a window
    python coph.py --headless --frames 600         # no GL: the readouts, printed at the end
    python coph.py --offscreen --frames 300 --png coph.png
    python coph.py --no-packets                    # each agent learns alone

In a browser on ariglx-web the page runs it in place of the document (?src=coph.py, with coph.ari
preloaded): Host() adopts the page's host, host.run() returns at once, and the frames run after.

The world is the crossing demo's (GRL-SNAM's crossing_demo, libcvc's G12): corridors along a
surveyed network, rows at y = 0, 4 and 7.5 between columns at x = 0 and 12, and two doors, one in
the middle row and one in the top row. Agent 0 starts at (0, 4) for (12, 4), agent 1 the other
way. Each is a point mass in the exact flow at damping 0.85; its sensor reads the risk at 21
points within 1.25 m (with G12's noise) and sees the doors within it. Every 8 CoPH steps each agent
refits its energy and, with packets on, exports what it learned to the other
(docs/roadmap/NAV-COPH-ROADMAP.md, C5 and C6).
"""

import math
import os
import sys

import numpy as np
import pycvc

from pycvc.ari import Host, main_args

HERE = os.path.dirname(os.path.abspath(__file__))

#: The crossing demo's doors: slabs (x, y, half-thickness, hx, hy) across a corridor.
DOORS = [(9.5, 4.0, 0.1, 0.0, 0.75), (2.5, 7.5, 0.1, 0.0, 0.75)]
#: Its surveyed network (coph_crossing_network) and the corridors' radius about each edge.
NODES = [(0.0, 0.0), (0.0, 4.0), (0.0, 7.5), (12.0, 0.0), (12.0, 4.0), (12.0, 7.5)]
EDGES = [(0, 1), (1, 2), (1, 4), (2, 5), (0, 3), (3, 4), (4, 5)]
CORRIDOR = 0.85
AGENTS = [{"initial": (0.0, 4.0, 0.0, 0.0), "goal": (12.0, 4.0)},
          {"initial": (12.0, 4.0, 0.0, 0.0), "goal": (0.0, 4.0)}]  # fmt: skip
SEEDS = [152000, 152001]
DT = 0.12  # CoPH's step (s)
#: The picture: the world's bounds (m) and its pixels per metre.
BOUNDS = (-1.2, -1.2, 13.2, 8.7)
PX = 38
COLORS = [(70, 150, 255), (255, 170, 40)]


def noise(agent, tick, j):
    """G12's sensor noise: deterministic, within +-0.003."""
    return 0.003 * (((7 * tick + 13 * j + 5 * agent) % 11) - 5) / 5.0


def _grid():
    w = int(round((BOUNDS[2] - BOUNDS[0]) * PX))
    h = int(round((BOUNDS[3] - BOUNDS[1]) * PX))
    xs = BOUNDS[0] + (np.arange(w) + 0.5) / PX
    ys = BOUNDS[3] - (np.arange(h) + 0.5) / PX  # row 0 is the top (north)
    return np.meshgrid(xs, ys)


def _segment_distance(X, Y, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    t = np.clip(((X - ax) * dx + (Y - ay) * dy) / max(dx * dx + dy * dy, 1e-12), 0.0, 1.0)
    return np.hypot(X - (ax + t * dx), Y - (ay + t * dy))


def background():
    """The maze: dark ground, the corridors light, the doors red."""
    X, Y = _grid()
    img = np.empty(X.shape + (3,), np.uint8)
    img[:] = (40, 44, 52)
    for e in EDGES:
        img[_segment_distance(X, Y, NODES[e[0]], NODES[e[1]]) <= CORRIDOR] = (214, 216, 206)
    for x, y, r, hx, hy in DOORS:
        img[_segment_distance(X, Y, (x - hx, y - hy), (x + hx, y + hy)) <= r + 0.03] = (210, 60, 50)
    return img


def _pixel(x, y):
    return int((BOUNDS[3] - y) * PX), int((x - BOUNDS[0]) * PX)


def _disc(img, x, y, radius, color, ring=0.0):
    """A filled disc of `radius` m (or, with `ring` > 0, a ring that wide) at (x, y)."""
    r, c = _pixel(x, y)
    k = int(radius * PX) + 2
    r0, r1, c0, c1 = max(r - k, 0), min(r + k + 1, img.shape[0]), max(c - k, 0), min(c + k + 1, img.shape[1])
    if r0 >= r1 or c0 >= c1:
        return
    rr, cc = np.mgrid[r0:r1, c0:c1]
    d = np.hypot(rr - r, cc - c) / PX
    mask = (d <= radius) & (d >= radius - ring) if ring > 0 else d <= radius
    img[r0:r1, c0:c1][mask] = color


def _line(img, a, b, color):
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * PX) + 1
    for t in np.linspace(0.0, 1.0, n):
        r, c = _pixel(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        if 0 <= r < img.shape[0] and 0 <= c < img.shape[1]:
            img[r, c] = color


class Crossing:
    """One run of the crossing: the fleet, its world, and what the picture and plots keep."""

    def __init__(self, packets=True, steps=2500):
        self.packets = bool(packets)
        self.fleet = pycvc.NavCophFleet({"steps": int(steps)}, {}, AGENTS,
                                        senders=[0, 1] if packets else [], seed=7)  # fmt: skip
        self.world = {"obstacles": [list(d) for d in DOORS], "seeds": SEEDS, "noise": noise}
        self.started = self.done = False
        self.trails = [[], []]
        self.dist = [[], []]
        self.flash = 0
        self.seen = 0
        self.sent = self.delivered = self.accepted = 0
        self.finish = [None, None]

    def step(self, ticks):
        """Run `ticks` more CoPH ticks (all of them until both agents are done)."""
        if self.done:
            return
        if self.started:
            self.done = self.fleet.run_crossing(ticks=ticks)
        else:
            self.done = self.fleet.run_crossing(self.world, ticks)
            self.started = True
        t = self.fleet.tick() * DT
        for i in range(2):
            x = self.fleet.state(i)
            self.trails[i].append((x[0], x[1]))
            g = AGENTS[i]["goal"]
            self.dist[i].append(math.hypot(g[0] - x[0], g[1] - x[1]))
            if self.fleet.done(i) and self.finish[i] is None:
                self.finish[i] = t
        events = self.fleet.events()
        fresh, self.seen = events[self.seen:], len(events)
        for e in fresh:
            self.sent += e["kind"] == "sent"
            if e["kind"] == "delivered":
                self.delivered += 1
                self.accepted += bool(e["accepted"])
                self.flash = 8
        self.flash = max(0, self.flash - 1)

    def picture(self, base):
        img = base.copy()
        for i in range(2):
            for x, y in self.trails[i][::2]:
                _disc(img, x, y, 0.035, COLORS[i])
            g = AGENTS[i]["goal"]
            _disc(img, g[0], g[1], 0.32, COLORS[i], ring=0.06)
        if self.flash and self.packets:
            _line(img, self.trails[0][-1], self.trails[1][-1], (120, 230, 140))
        for i in range(2):
            x, y = self.trails[i][-1] if self.trails[i] else AGENTS[i]["initial"][:2]
            _disc(img, x, y, 1.25, tuple(int(0.6 * c) for c in COLORS[i]), ring=0.04)
            _disc(img, x, y, 0.16, COLORS[i])
        return img

    def status(self):
        if not self.started:
            return "starting"
        words = []
        for i in range(2):
            if self.fleet.hit(i):
                words.append(f"agent {i} hit")
            elif self.fleet.success(i):
                words.append(f"agent {i} at its goal ({self.finish[i]:.1f} s)")
            elif self.fleet.done(i):
                words.append(f"agent {i} out of steps")
        return "; ".join(words) or "running"

    def summary(self, i):
        a = self.fleet.agent_state(i)
        samples = self.fleet.samples(i)
        stage = samples[-1]["stage"] if samples else 0
        return (f"stage {stage}, version {a['version']}, damping {a['damping']:.3f}, "
                f"policy ({a['policy'][0]:.2f}, {a['policy'][1]:.2f})")  # fmt: skip

    def energy(self, i):
        e = self.fleet.energy(i)
        return f"work {e['work']:.3f}, heat {e['heat']:.3f}, residual {e['residual']:.1e}"


def main(argv=None):
    args = main_args(argv)
    rest = list(args.rest)
    packets = "--no-packets" not in rest
    if not hasattr(pycvc, "NavCophFleet"):
        sys.exit("coph.py needs pycvc with NavCophFleet (libcvc's CoPH bindings, C6)")
    host = Host(mode=args.mode, name="coph", width=args.width, height=args.height)
    host.load(os.path.join(HERE, "coph.ari"))
    if not packets:
        host.set("run.packets", "0")
    base = background()
    st = {"run": Crossing(packets)}

    def restart():
        st["run"] = Crossing(host.get_bool("run.packets", True))

    host.on("restart", restart)
    host.watch("run.packets", lambda _v: restart())

    def publish():
        run = st["run"]
        host.set_many({"time": f"{run.fleet.tick() * DT:.1f}",
                       "packets": f"{run.sent} sent, {run.delivered} delivered ({run.accepted} "
                                  "accepted)" if run.packets else "off: each learns alone",
                       "status": run.status()}, prefix="stats")  # fmt: skip
        for i in range(2):
            host.set_many({"summary": run.summary(i), "energy": run.energy(i)}, prefix=f"agent{i}")
            host.set_series(f"dist{i}", run.dist[i][-400:])
        host.publish_image("crossing", run.picture(base))

    @host.on_frame
    def tick(dt):
        run = st["run"]
        if not host.get_bool("run.paused"):
            run.step(max(1, host.get_int("run.ticks", 4)))
        publish()

    publish()
    rc = host.run(fps=30, frames=args.frames or None)
    if getattr(host, "embedded", False):  # a browser page: the frames run after this returns
        return rc
    if args.png:
        host.snapshot(args.png)
    run = st["run"]
    print(f"coph: {run.fleet.tick() * DT:.1f} s, {run.status()}; packets "
          f"{run.sent} sent, {run.delivered} delivered ({run.accepted} accepted)")  # fmt: skip
    for i in range(2):
        print(f"  agent {i}: {run.summary(i)}; {run.energy(i)}")
    code = host.exit_code()
    host.close()
    return code


if __name__ == "__main__":
    sys.exit(main())
