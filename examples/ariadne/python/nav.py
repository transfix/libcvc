"""nav.py — a Python loop drives a fleet of Humvees through a city on a cvc::nav sim_world (pycvc)
and draws it in nav.ari.

    python nav.py                                   # a window
    python nav.py --plant kinematic --agents 12
    python nav.py --bundle DIR                      # a scene bundle (terrain.json + buildings.glb)
    python nav.py --offscreen --png nav.png --frames 300

nav.ari draws a scene, so it requires ``gl.scene``: there is no ``--headless`` run (``--offscreen``
draws without a window).

The world is Austin when its scene bundle is installed (``cvcpkg install scene-austin-south-small
vehicle-humvee``; found by pycvc_gl.scenes.find_scene, or ``--bundle`` / ``$CVC_NAV_BUNDLE``),
cropped to ``--half`` metres around its centre; without one it is a block city, so the example
runs anywhere. Each agent is a Humvee at its real size (the ``Humvee.glb`` model when found, else
a box of the physics preset's size). It follows a route planned for its turning radius
(pycvc.nav_plan_vehicle_route, driven by a route follower) to a random goal in the streets, and a
new one when it arrives. The plant is Jolt physics (``pycvc.nav_plant``: the Humvee's engine,
tyres and suspension, on the city's terrain and buildings) or nav's kinematic bicycle; the
document's Plant combo switches between them.

Every frame: the Sim window's controls are read, the world is stepped ``sim.ticks`` times, the
Humvees, routes and camera move, and the readouts and the map picture are published.
"""

import argparse
import math
import os
import sys

import numpy as np
import pycvc
import pycvc_gl

from pycvc_gl.ariadne import Host, main_args

HERE = os.path.dirname(os.path.abspath(__file__))
SCALE = 0.05  # the navigator's normalized units per metre: its trained metric, as the C++ demos
CELL_M = 3.0  # the planning grid's cell
BLOCK_HALF = 300.0  # the block city's half-size (m)
HUMVEE_LEN = 4.72  # Humvee.glb's real length (m)


def has_jolt():
    return (hasattr(pycvc, "nav_plant") and "jolt" in pycvc.physics_backends()
            and pycvc.physics_backend_info("jolt")["available"])


# ── the world ──────────────────────────────────────────────────────────────────────────────────


class World:
    """The city: its occupancy grid (what routes are planned on), bounds, terrain sampler, and
    the meshes the scene and the physics world share."""

    def __init__(self, app, bundle, half):
        self.bundle = bundle
        if bundle:
            city = pycvc.load_city_bundle(app, bundle)
            mnx, mny, mxx, mxy = city.bounds2d()
            half = min(half, 0.5 * (mxx - mnx), 0.5 * (mxy - mny))
            cx, cy = 0.5 * (mnx + mxx), 0.5 * (mny + mxy)
            self.bounds = (cx - half, cy - half, cx + half, cy + half)
            self.n = int(round(2 * half / CELL_M)) + 1
            planes = pycvc.nav_world_rasterize(city.world(), self.n, self.n, *self.bounds)
            self.occ = planes["occupancy"]
            self.heights = planes["height"]  # the terrain at each grid sample (m)
            self.field = city.terrain()
            self.buildings = city.world().item_geometry("buildings") if city.buildings_loaded \
                else None
            self.surfaces = max(city.world().surfaces_used(), default=0) + 1  # the ids in use
            self.name = os.path.basename(os.path.normpath(bundle))
        else:
            half = BLOCK_HALF
            self.bounds = (-half, -half, half, half)
            self.n = int(round(2 * half / CELL_M)) + 1
            self.occ, self.blocks = block_city(self.n, self.bounds)
            self.heights = None
            self.field = None
            self.buildings = None
            self.name = "block city"
        border(self.occ)
        self.cx = 0.5 * (self.bounds[0] + self.bounds[2])
        self.cy = 0.5 * (self.bounds[1] + self.bounds[3])
        # The grade, for the Humvee: its steepest climb on mu 0.9 and its static stability
        # (nav_plant.terrain_config_of). The navigator takes the terrain into its energy
        # (Fleet: nav_sim_world_set_terrain), and the routes keep off the slopes past its limit.
        self.terrain_config = None
        self.steep = np.zeros(self.occ.shape, bool)
        if self.heights is not None and hasattr(pycvc, "nav_terrain_build"):
            self.terrain_config = pycvc.nav_plant.terrain_config_of(pycvc.vehicle_desc.humvee(),
                                                                    0.9)
            cell = (self.bounds[2] - self.bounds[0]) / (self.n - 1)
            self.steep = pycvc.nav_terrain_build(self.heights, cell, self.terrain_config)[
                "infeasible"].astype(bool)
        self.plan_occ = (self.occ.astype(bool) | self.steep).astype(np.uint8)
        # Where a Humvee fits: the free, drivable cells at least 2 cells (6 m) from a building.
        self.roomy = ~dilate(self.plan_occ.astype(bool), 2)

    def height(self, x, y):
        if self.field is None:
            return 0.0
        h = self.field.height(float(x), float(y))
        return 0.0 if math.isnan(h) else h

    def cell(self, x, y):
        mnx, mny, mxx, mxy = self.bounds
        c = int(round((x - mnx) / (mxx - mnx) * (self.n - 1)))
        r = int(round((y - mny) / (mxy - mny) * (self.n - 1)))
        return min(max(r, 0), self.n - 1), min(max(c, 0), self.n - 1)

    def point(self, r, c):
        mnx, mny, mxx, mxy = self.bounds
        return mnx + c / (self.n - 1) * (mxx - mnx), mny + r / (self.n - 1) * (mxy - mny)

    def random_free(self, rng, away_from=None, min_dist=0.0):
        """A random roomy street point, at least ``min_dist`` m from ``away_from``."""
        rr, cc = np.nonzero(self.roomy)
        for _ in range(200):
            k = int(rng.integers(len(rr)))
            x, y = self.point(rr[k], cc[k])
            if away_from is None or math.hypot(x - away_from[0], y - away_from[1]) >= min_dist:
                return x, y
        return self.point(rr[0], cc[0])


def block_city(n, bounds, block=48.0, street=24.0):
    """A grid of blocks (with a few left open as squares): occupancy and the blocks' boxes."""
    occ = np.zeros((n, n), np.uint8)
    mnx, mny, mxx, mxy = bounds
    pitch = block + street
    blocks = []
    k = 0
    x0 = mnx + street
    while x0 + block < mxx - street:
        y0 = mny + street
        while y0 + block < mxy - street:
            k += 1
            if k % 7 != 3:  # every seventh lot is a square
                h = 12.0 + 26.0 * ((k * 37) % 11) / 10.0
                blocks.append((x0, y0, x0 + block, y0 + block, h))
                c0 = int((x0 - mnx) / (mxx - mnx) * (n - 1))
                c1 = int(math.ceil((x0 + block - mnx) / (mxx - mnx) * (n - 1)))
                r0 = int((y0 - mny) / (mxy - mny) * (n - 1))
                r1 = int(math.ceil((y0 + block - mny) / (mxy - mny) * (n - 1)))
                occ[r0:r1 + 1, c0:c1 + 1] = 1
            y0 += pitch
        x0 += pitch
    return occ, blocks


def footprint_boxes(occ, bounds):
    """The occupied cells as few axis-aligned rectangles (x0, y0, x1, y1): each row's runs, grown
    down while the next rows have the same run."""
    rows, cols = occ.shape
    mnx, mny, mxx, mxy = bounds
    cw, ch = (mxx - mnx) / (cols - 1), (mxy - mny) / (rows - 1)
    done = np.zeros(occ.shape, bool)
    out = []
    for r in range(rows):
        c = 0
        while c < cols:
            if not occ[r, c] or done[r, c]:
                c += 1
                continue
            c1 = c
            while c1 + 1 < cols and occ[r, c1 + 1] and not done[r, c1 + 1]:
                c1 += 1
            r1 = r
            while r1 + 1 < rows and occ[r1 + 1, c:c1 + 1].all() and not done[r1 + 1, c:c1 + 1].any():
                r1 += 1
            done[r:r1 + 1, c:c1 + 1] = True
            out.append((mnx + (c - 0.5) * cw, mny + (r - 0.5) * ch, mnx + (c1 + 0.5) * cw,
                        mny + (r1 + 0.5) * ch))
            c = c1 + 1
    return out


def border(occ):
    occ[0, :] = occ[-1, :] = 1
    occ[:, 0] = occ[:, -1] = 1


def dilate(mask, k):
    out = mask.copy()
    for dr in range(-k, k + 1):
        for dc in range(-k, k + 1):
            out |= np.roll(np.roll(mask, dr, 0), dc, 1)
    return out


def boxes_mesh(boxes):
    """(x0, y0, x1, y1, height) boxes as one flat (vertices, triangles) mesh."""
    verts, tris = [], []
    for x0, y0, x1, y1, h in boxes:
        b = len(verts) // 3
        for z in (0.0, h):
            verts += [x0, y0, z, x1, y0, z, x1, y1, z, x0, y1, z]
        for f in ((4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7),
                  (2, 7, 6), (3, 0, 4), (3, 4, 7)):
            tris += [b + f[0], b + f[1], b + f[2]]
    return [float(v) for v in verts], tris


# ── the fleet ──────────────────────────────────────────────────────────────────────────────────


class Fleet:
    """n Humvees on a sim_world over the world's grid, on a plant, each following a route."""

    def __init__(self, world, n, plant, rng, relief=False):
        self.world, self.n, self.rng, self.relief = world, n, rng, relief
        starts = [world.random_free(rng) for _ in range(n)]
        goals = [world.random_free(rng, s, 150.0) for s in starts]
        to_norm = lambda p: ((p[0] - world.cx) * SCALE, (p[1] - world.cy) * SCALE)  # noqa: E731
        on = np.array([to_norm(p) for p in starts], np.float32)
        goal = np.array([to_norm(p) for p in goals], np.float32)
        occ = world.occ
        desc = pycvc.vehicle_desc.humvee()
        self.desc = desc
        # The navigator's limits in its normalized units (metres * SCALE): radius 1.2 m, barrier
        # 3 m, top speed 9 m/s, the Humvee's 3.3 m wheelbase and its steering lock; the map is
        # known (prior = truth) and not sensed, since the routes are planned on it.
        self.sim = pycvc.nav_sim_world_create(
            occ, occ, "", on, goal, np.ones((n, 3), np.float32), world.n, world.n,
            *world.bounds, world.cx, world.cy, SCALE, 60.0, 120, 2 * math.pi,
            1.2 * SCALE, 3.0 * SCALE, 0.06, 9.0 * SCALE, 3.3 * SCALE, float(desc.max_steer),
            2.4 * SCALE, 3.0 * SCALE, 0.8, 1, 1, 6.0 * SCALE, 4, 1, 2.2, -1.4, 8.0, 1, 0.5, 0.15,
            4.0)
        if world.terrain_config is not None:  # the grade, in the navigator's energy
            pycvc.nav_sim_world_set_terrain(self.sim, world.heights, world.terrain_config)
        self.plant_kind = "kinematic"
        self.physics = self.plant = None
        if plant == "jolt":
            self._attach_jolt()
        rv = pycvc.nav_sim_world_route_vehicle(self.sim, 0)
        if rv is None:  # nav's bicycle: the Humvee's turning radius and body, from its preset
            wb, steer = 3.3, float(desc.max_steer)
            rv = {"min_radius": wb / math.tan(steer), "disc_offsets": [0.0, 2.4],
                  "disc_radius": 1.3, "wheelbase": wb, "max_steer": steer}
        self.route_vehicle = {k: rv[k] for k in ("min_radius", "disc_offsets", "disc_radius")}
        self.follower = pycvc.nav_route_follower_create(
            self.sim, {"wheelbase": rv["wheelbase"], "max_steer": rv["max_steer"]})
        self.routes = [np.zeros((0, 4))] * n
        self.trips = 0
        self.tick = 0
        for i in range(n):
            self.new_route(i, goals[i])

    def _attach_jolt(self):
        """The Jolt world the Humvees drive in. By default its ground is flat and the buildings are
        boxes over their footprints (as the C++ city demos run it); the scene still draws each
        Humvee seated on the terrain. With `relief`, the physics has the city's own terrain and
        building mesh. The navigator has the grade either way (World.terrain_config) and the routes
        keep off slopes past the Humvee's limit, but that is its climb, not its cornering: on
        Austin's relief some Humvees still roll over taking a 10-25 degree side slope at 5-9 m/s
        (4 of 16 in 96 s here), so the flat ground is the default."""
        w = pycvc.physics_world(backend="jolt", fixed_dt=0.01)
        ground = w.add_material("ground", friction=0.9, rolling_resistance=0.015)
        wall = w.add_material("building", friction=0.6)
        world = self.world
        relief = self.relief and world.field is not None
        if relief:  # the terrain's surfaces keep their own materials
            w.add_terrain(world.field)
            if world.buildings is not None:  # each surface id the city uses -> building
                w.add_mesh(world.buildings, [wall] * world.surfaces)
        else:
            mnx, mny, mxx, mxy = world.bounds
            w.add_box((0.5 * (mxx - mnx) + 50, 0.5 * (mxy - mny) + 50, 0.5),
                      position=(world.cx, world.cy, -0.5), motion="static", material=ground)
            for x0, y0, x1, y1 in footprint_boxes(world.occ, world.bounds):
                w.add_box((0.5 * (x1 - x0), 0.5 * (y1 - y0), 10.0),
                          position=(0.5 * (x0 + x1), 0.5 * (y0 + y1), 10.0), motion="static",
                          material=wall)
        self.physics = w
        # On the relief each Humvee starts on the ground under it, not at z = 0.
        self.plant = pycvc.nav_plant(w, [self.desc] * self.n, latency=0.1, brake_margin=0.8,
                                     ground=world.field if relief else None)
        pycvc.nav_sim_world_set_plant(self.sim, self.plant)
        self.plant_kind = pycvc.nav_sim_world_plant_kind(self.sim)

    def new_route(self, i, goal=None):
        """Plan agent i a route from where it is to `goal` (a random street point by default)."""
        pos, head = pycvc.nav_sim_world_snapshot(self.sim)[:2]
        x, y, h = float(pos[i, 0]), float(pos[i, 1]), float(head[i])
        for _ in range(6):
            g = goal or self.world.random_free(self.rng, (x, y), 150.0)
            r = pycvc.nav_plan_vehicle_route(self.world.plan_occ, *self.world.bounds, x, y, h, *g,
                                             self.route_vehicle)
            if r["reached"] and len(r["poses"]) >= 2:
                pycvc.nav_route_follower_set(self.follower, i, r["poses"])
                self.routes[i] = r["poses"]
                return True
            goal = None
        return False

    def step(self, ticks):
        for _ in range(ticks):
            pycvc.nav_route_follower_apply(self.follower)
            pycvc.nav_sim_world_step(self.sim)
            self.tick += 1
        done = pycvc.nav_route_follower_status(self.follower)["done"]
        for i in np.flatnonzero(done):
            self.trips += 1
            self.new_route(int(i))

    def shuffle(self):
        for i in range(self.n):
            self.new_route(i)


# ── drawing ────────────────────────────────────────────────────────────────────────────────────


def humvee_model(host, world):
    """The Humvee as a pycvc_gl.vehicle.VehicleModel, or None (then a box of its size)."""
    from pycvc_gl.scenes import find_vehicle_model
    from pycvc_gl.vehicle import VehicleModel

    path = find_vehicle_model(world.bundle)
    return VehicleModel(host.app, path, HUMVEE_LEN) if path else None


def map_picture(world, fleet, show_routes, size=210):
    """A top-down map: streets light, buildings dark, routes amber, Humvees cyan (row 0 north)."""
    occ = world.occ[::-1]
    img = np.where(occ[..., None] > 0, [52, 56, 66], [196, 200, 190]).astype(np.uint8)
    pos = pycvc.nav_sim_world_snapshot(fleet.sim)[0]

    def put(x, y, color, r):
        rr, cc = world.cell(x, y)
        rr = world.n - 1 - rr
        img[max(rr - r, 0):rr + r + 1, max(cc - r, 0):cc + r + 1] = color

    if show_routes:
        for route in fleet.routes:
            for x, y in route[::4, :2]:
                put(x, y, (235, 160, 40), 0)
    for x, y in pos:
        put(float(x), float(y), (40, 200, 255), 2)
    k = max(1, world.n // size)
    return img[::k, ::k]


def route_lines(app, world, fleet, z=0.6):
    g = pycvc.geometry(app)
    verts, lines = [], []
    for route in fleet.routes:
        pts = route[:, :2]
        if len(pts) < 2:
            continue
        b = len(verts) // 3
        for x, y in pts:
            verts += [float(x), float(y), world.height(x, y) + z]
        lines += [v for k in range(len(pts) - 1) for v in (b + k, b + k + 1)]
    if verts:
        g.add_vertices(verts)
        g.add_lines(lines)
    return g


def main(argv=None):
    args = main_args(argv)
    p = argparse.ArgumentParser()
    p.add_argument("--bundle", default=None, help="a scene bundle dir (default: Austin if found)")
    p.add_argument("--no-bundle", action="store_true", help="the block city even with Austin")
    p.add_argument("--plant", choices=("auto", "jolt", "kinematic"), default="auto")
    p.add_argument("--agents", type=int, default=8)
    p.add_argument("--camera", choices=("follow", "overview"), default="follow")
    p.add_argument("--half", type=float, default=600.0, help="Austin's crop half-size (m)")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--relief", action="store_true",
                   help="Jolt on the city's terrain and building mesh (some Humvees roll over)")
    extra = p.parse_args(args.rest)
    if args.mode == "headless":
        sys.exit("nav.py: nav.ari draws a scene (requires: gl.scene); run it in a window or "
                 "--offscreen")

    from pycvc_gl.scenes import find_scene

    bundle = None if extra.no_bundle else (extra.bundle or find_scene())
    plant = extra.plant if extra.plant != "auto" else ("jolt" if has_jolt() else "kinematic")
    host = Host(mode=args.mode, name="nav", width=args.width, height=args.height)
    host.load(os.path.join(HERE, "nav.ari"))  # its warnings are logged
    host.set_many({"agents": extra.agents, "plant": plant, "camera": extra.camera}, prefix="sim")
    rng = np.random.default_rng(extra.seed)
    world = World(host.app, bundle, extra.half)
    model = humvee_model(host, world)
    st = {"fleet": None, "speeds": [], "nodes": 0}

    def restart():
        n = max(1, host.get_int("sim.agents", 8))
        kind = host.get("sim.plant", "jolt")
        if kind == "jolt" and not has_jolt():
            kind = "kinematic"
        st["fleet"] = Fleet(world, n, kind, rng, relief=extra.relief)
        st["speeds"] = []
        for i in range(n, st["nodes"]):
            host.scene.removeGraphics(f"humvee_{i}")
        for i in range(st["nodes"], n):
            if model is not None:
                model.add(host.scene, f"humvee_{i}")
            else:
                host.lab.add_mesh(f"humvee_{i}", *boxes_mesh([(-2.42, -1.09, 2.42, 1.09, 1.9)]),
                                  color=(0.42, 0.45, 0.33))
        st["nodes"] = n

    host.on("restart", restart)
    host.on("shuffle", lambda: st["fleet"].shuffle())

    @host.watch("sim.plant")
    @host.watch("sim.agents")
    def changed(_value):
        if st["fleet"] is not None:
            restart()

    # the city, under the document's lights
    if world.bundle:
        from pycvc_gl.scenes import load_geometry_bundle

        load_geometry_bundle(host.lab, world.bundle, terrain_color=(0.42, 0.46, 0.36),
                             building_color=(0.80, 0.78, 0.74))
        for name, (amb, dif, spec) in (("terrain", (0.4, 0.85, 0.0)),
                                       ("buildings", (0.3, 0.8, 0.04))):
            gn = host.scene.geometry_node(name)
            gn.setAmbient(amb)
            gn.setDiffuse(dif)
            gn.setSpecular(spec)
    else:
        mnx, mny, mxx, mxy = world.bounds
        host.lab.add_mesh("ground", [mnx, mny, 0, mxx, mny, 0, mxx, mxy, 0, mnx, mxy, 0],
                          [0, 1, 2, 0, 2, 3], color=(0.42, 0.46, 0.36))
        host.lab.add_mesh("buildings", *boxes_mesh(world.blocks), color=(0.80, 0.78, 0.74))
    host.scene.addGraphics("routes", pycvc.geometry(host.app))
    host.set_many({"back": 2.5 * HUMVEE_LEN, "height": 1.2 * HUMVEE_LEN,
                   "look_ahead": 1.2 * HUMVEE_LEN, "look_up": 0.4 * HUMVEE_LEN,
                   "cam_tau": 0.15, "pos_tau": 0.1}, prefix="viewers.main.camera.track")
    restart()

    @host.on_frame
    def tick(dt):
        fleet = st["fleet"]
        if not host.get_bool("sim.paused"):
            fleet.step(max(1, host.get_int("sim.ticks", 2)))
        pos, head, speed = pycvc.nav_sim_world_snapshot(fleet.sim)[:3]
        st["speeds"] = (st["speeds"] + [float(np.mean(np.abs(speed)))])[-200:]
        host.set_many({"world": world.name, "plant": fleet.plant_kind,
                       "time": f"{fleet.tick * 0.06:.1f}", "trips": fleet.trips,
                       "speed": f"{st['speeds'][-1]:.2f}"}, prefix="stats")
        host.set_series("speeds", st["speeds"])
        summary = f"{abs(float(speed[0])):.1f} m/s"
        if fleet.plant is not None:
            vs = fleet.physics.vehicle_state(fleet.plant.vehicle(0))
            summary += f", gear {vs['gear']}, {vs['engine_rpm']:.0f} rpm"
        host.set("agent.summary", summary)
        if host.frame_count % 5 == 0:
            host.publish_image("map", map_picture(world, fleet, host.get_bool("show.routes")))
        from pycvc_gl.vehicle import body_transform, seat_transform

        for i in range(fleet.n):
            x, y = float(pos[i, 0]), float(pos[i, 1])
            if fleet.plant is not None and fleet.relief:  # the chassis' own pose, on the ground
                s = fleet.physics.vehicle_state(fleet.plant.vehicle(i))
                m = body_transform(s["position"], s["rotation"], world.height(x, y))
            else:  # its heading, tilted to the terrain under it
                m = seat_transform(x, y, float(head[i]), world.height if world.field else None)
            host.scene.getGraphics(f"humvee_{i}").setTransform(m)
        routes = host.scene.geometry_node("routes")
        routes.setGeometry(route_lines(host.app, world, fleet) if host.get_bool("show.routes")
                           else pycvc.geometry(host.app))
        routes.setRenderMode(pycvc_gl.GeometryRenderMode_LINES)
        routes.setLineWidth(2.0)
        routes.setUseSingleColor(True)
        routes.setColor(0.95, 0.65, 0.15)
        x0, y0 = float(pos[0, 0]), float(pos[0, 1])
        if host.get("sim.camera") == "follow":
            host.camera.setMode(pycvc_gl.CameraController.Mode_Track)
            host.camera.feedTrackTarget(x0, y0, world.height(x0, y0))
        elif st.get("camera") != "overview":
            mnx, mny, mxx, mxy = world.bounds
            host.camera.setMode(pycvc_gl.CameraController.Mode_Orbit)
            host.frame_bounds((mnx, mny, 0.0), (mxx, mxy, 40.0))
        st["camera"] = host.get("sim.camera")

    host.run(fps=30, frames=args.frames or None)
    if args.png:
        host.snapshot(args.png)
    print("world:", host.get("stats.world"), "plant:", host.get("stats.plant"), "trips:",
          host.get("stats.trips"), "sim time:", host.get("stats.time"))
    code = host.exit_code()
    host.close()
    return code


if __name__ == "__main__":
    sys.exit(main())
