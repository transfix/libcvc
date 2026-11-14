"""pycvc_gl.cars: N Ackermann cars that a controller of yours drives, shown in an Ariadne document.

    python -m pycvc_gl.cars                        # Austin, 6 Humvees, the template's controller
    python -m pycvc_gl.cars --copy my_cars         # cars.py and cars.ari, to make your own
    python my_cars/cars.py --headless --frames 600 --log run.npz

The template (``cars_template.py``, copied as ``cars.py``) is a controller and its parameters;
this module is everything under it:

- :class:`World`, where the cars drive: a city from the document's ``data:`` block (the
  ``scene-austin-south-small`` package by default, or any package or local bundle), a world from
  libcvc's terrain generator (:func:`generate`), or a flat walled square. Each gives the cars the
  same things: an occupancy grid, a terrain height, and the geometry the physics collides with.
- :class:`Cars`: libcvc's ``sim_world`` over that grid, its agents on a plant that
  ``pycvc.plant_host`` picks by name (nav's kinematic bicycle, Jolt vehicles, the dynamic single
  track), and your ``(steer, accel)`` in its navigator's place (``nav_sim_world_override``). With
  ``navigator=True`` the navigator drives instead, as a baseline.
- :class:`GymCars`: the same cars in a world document (``pycvc.WorldGym``, ARIADNE_WORLDS.md
  §14, L4): the document's world, its vehicles as Jolt vehicles in its own physics world, its
  spawn points. Your controller runs on it unchanged; ``--gym DOC`` runs the template on it.
- :class:`CarsState`: what a controller reads each tick, as numpy arrays (SI units).
- :func:`rollout`: a controller's own predicted paths, on nav's bicycle.
- :func:`local_raster`: the occupancy and signed distance around a car, heading up.
- :class:`CarsLog`: every tick's state and command, the rasters and the settings, to .npz or .csv.
- :func:`run`: the Ariadne host the template calls (panels, sliders, callbacks, headless).

A host of your own reuses the rest: the panels as units (``cars_panels.ari``, imported by
``cvc://`` with :data:`COMPONENTS` as a component path), the flags and settings
(:func:`add_args`, :func:`apply_settings`), the worlds (:func:`make_world`, :func:`show_world`),
a state from cars stepped some other way (:func:`state_from`), the pictures and plots
(:func:`raster_arrays`, :func:`raster_pictures`, :func:`plot_coefficients`) and the end of a run
(:func:`save_log`, :func:`exit_code`). docs/CARS.md, "Your own host".

Every navigator, vehicle and plant setting is a state-tree node the document can set in its
``init:``, bind to a slider, or that Python sets: libcvc's own keys, ``<prefix>.nav.*`` and
``<prefix>.nav.vehicle.*`` (``sim_world::pull``) and ``<prefix>.nav_plant.*`` (``plant_host``:
``vehicle`` switches the plant live). :func:`settings` lists them with their units and ranges.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

try:
    import pycvc
except Exception:  # pragma: no cover - pycvc is what drives the cars
    pycvc = None

#: The plants a world document runs its vehicles on (GymCars' ``vehicle``).
GYM_PLANTS = ("jolt", "kinematic", "dynamic", "dynamic4w")
#: The tick (s): the controller runs once per tick, and the world steps once.
DT = 0.05
#: The navigator's normalized units per metre (its trained metric, as libcvc's nav demos).
SCALE = 0.05
#: nav's bicycle substeps per tick.
NSUB = 5
#: The default world: the document's data.city (Austin), the generator, or a flat square.
WORLDS = ("data", "generated", "flat")
#: The package the template's data: block names.
AUSTIN = "cvcpkg://cy-pca/scene-austin-south-small"
AUSTIN_PATH = "share/cvc-scenes/austin_south"
#: The generator's settings the template reads (the generated_world node's world.* keys).
WORLD_KEYS = ("seed", "half", "trees", "rocks", "buildings", "tree_gen", "amp", "water_level",
              "preset", "ontology", "terrain", "city", "terrain_file", "edge")  # fmt: skip


def has_cars() -> bool:
    """This pycvc has what the cars need: nav_sim_world_override and plant_host (libcvc 3.7.12+)."""
    return pycvc is not None and all(
        hasattr(pycvc, k)
        for k in ("nav_sim_world_override", "nav_sim_world_set_pose", "plant_host")
    )


def _require():
    if not has_cars():
        raise RuntimeError(
            "pycvc_gl.cars needs pycvc with nav_sim_world_override and plant_host (libcvc 3.7.12+): "
            "cvcpkg install pycvc-gl-cp312 (or your Python's column)"
        )


# ── the world ──────────────────────────────────────────────────────────────────────────────────


@dataclass
class World:
    """Where the cars drive.

    ``occ`` is the occupancy grid (rows, cols; row 0 at ``min_y``, cell centres on the grid points)
    over ``bounds`` (min_x, min_y, max_x, max_y, m): buildings, hard ground, deep water. ``height``
    samples the terrain (None: flat at 0). ``geometry`` is the ``pycvc.world_geometry`` the physics
    collides with (None: a flat ground). ``kind`` is ``data``, ``generated`` or ``flat``; ``info``
    says where it came from (a URI, the generator's params). ``boxes`` are the flat square's walls,
    for drawing. ``building_height`` is how tall an occupied cell stands in a physics box."""

    kind: str
    occ: np.ndarray
    bounds: tuple
    height: object = None
    geometry: object = None
    info: dict = field(default_factory=dict)
    boxes: list = field(default_factory=list)
    _sdf: np.ndarray | None = None

    @property
    def cell(self) -> float:
        return (self.bounds[2] - self.bounds[0]) / (self.occ.shape[1] - 1)

    @property
    def center(self) -> tuple:
        return 0.5 * (self.bounds[0] + self.bounds[2]), 0.5 * (self.bounds[1] + self.bounds[3])

    @property
    def span(self) -> float:
        return min(self.bounds[2] - self.bounds[0], self.bounds[3] - self.bounds[1])

    def cells(self, x, y):
        """Grid (row, col) of world points (arrays), clamped to the grid."""
        rows, cols = self.occ.shape
        c = np.clip(np.rint((np.asarray(x) - self.bounds[0]) / self.cell), 0, cols - 1).astype(int)
        r = np.clip(np.rint((np.asarray(y) - self.bounds[1]) / self.cell), 0, rows - 1).astype(int)
        return r, c

    def occupied(self, x, y):
        """Whether world points (arrays) are blocked or off the grid."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        mnx, mny, mxx, mxy = self.bounds
        out = (x < mnx) | (x > mxx) | (y < mny) | (y > mxy)
        r, c = self.cells(x, y)
        return out | (self.occ[r, c] > 0)

    @property
    def sdf(self) -> np.ndarray:
        """The signed distance to the nearest blocked cell (m): positive in the free cells,
        negative inside the blocked ones."""
        if self._sdf is None:
            free = np.ascontiguousarray(self.occ > 0, np.uint8)
            out = np.sqrt(np.asarray(pycvc.nav_edt2_squared(free), float))
            inside = np.sqrt(np.asarray(pycvc.nav_edt2_squared((1 - free).astype(np.uint8)), float))
            self._sdf = (np.where(free > 0, -inside, out)) * self.cell
        return self._sdf

    def z(self, x, y) -> float:
        if self.height is None:
            return 0.0
        h = self.height(float(x), float(y))
        return 0.0 if h is None or math.isnan(h) else float(h)

    def free_points(self, rng, n, clearance, min_sep, avoid=None, avoid_dist=0.0, near=None):
        """``n`` random points at least ``clearance`` m from anything blocked, ``min_sep`` m apart,
        ``avoid_dist`` m from the matching row of ``avoid`` and, with ``near`` ((x, y), r), within
        r of a point."""
        sdf = self.sdf
        rr, cc = np.nonzero(sdf >= clearance)
        if len(rr) == 0:
            rr, cc = np.nonzero(sdf > 0)
        if len(rr) == 0:
            raise RuntimeError(f"the {self.kind} world has no free ground for the cars")
        xs = self.bounds[0] + cc * self.cell
        ys = self.bounds[1] + rr * self.cell
        if near is not None:
            (nx, ny), nr = near
            keep = np.hypot(xs - nx, ys - ny) <= nr
            if keep.sum() >= n:
                xs, ys = xs[keep], ys[keep]
        pts = []
        for i in range(n):
            best = None
            for _ in range(400):
                k = int(rng.integers(len(xs)))
                p = (float(xs[k]), float(ys[k]))
                if any(math.hypot(p[0] - q[0], p[1] - q[1]) < min_sep for q in pts):
                    continue
                if (
                    avoid is not None
                    and math.hypot(p[0] - avoid[i][0], p[1] - avoid[i][1]) < avoid_dist
                ):
                    best = best or p
                    continue
                best = p
                break
            pts.append(best or (float(xs[0]), float(ys[0])))
        return np.array(pts, float)


def _border(occ):
    occ[0, :] = occ[-1, :] = 1
    occ[:, 0] = occ[:, -1] = 1


def _rasterize(geometry, bounds, cell):
    n = int(round((bounds[2] - bounds[0]) / cell)) + 1
    m = int(round((bounds[3] - bounds[1]) / cell)) + 1
    planes = pycvc.nav_world_rasterize(geometry, m, n, *bounds)
    occ = np.ascontiguousarray(planes["occupancy"], np.uint8)
    _border(occ)
    return occ


def world_from_data(app, path, *, half=None, cell=2.0) -> World:
    """The city a data store dataset holds (a ``city_bundle``: the document's ``data.city``, by
    its path ``<prefix>.data.city``), waiting for it to load. ``half`` crops it to a square that
    many metres around its centre."""
    _require()
    if not pycvc.data_wait(app, path, timeout=900.0):
        why = pycvc.data_status(app, path) or "not bound"
        try:
            why = (
                pycvc.state_get(
                    app, (pycvc.State.instance(app).child(path).linkTarget() or path) + ".error"
                )
                or why
            )
        except Exception:  # noqa: BLE001 - the error key is a nicety
            pass
        raise RuntimeError(f"the world at {path} did not load: {why}")
    geometry = pycvc.data_world(app, path)
    terrain = pycvc.data_height_field(app, path)
    mnx, mny, mxx, mxy = terrain.bounds2d()
    if half:
        cx, cy = 0.5 * (mnx + mxx), 0.5 * (mny + mxy)
        h = min(half, 0.5 * (mxx - mnx), 0.5 * (mxy - mny))
        mnx, mny, mxx, mxy = cx - h, cy - h, cx + h, cy + h
    bounds = (mnx, mny, mxx, mxy)
    uri = pycvc.state_get(app, path + ".uri") if pycvc.state_has(app, path + ".uri") else ""
    canonical = (
        pycvc.state_get(app, path + ".canonical")
        if pycvc.state_has(app, path + ".canonical")
        else ""
    )
    return World("data", _rasterize(geometry, bounds, cell), bounds, terrain.height, geometry,
                 {"source": "data", "uri": uri, "canonical": canonical, "path": path})  # fmt: skip


def building_library() -> str:
    """The building archetypes the generator uses, by the generated_world node's own rule outside
    wasm: ``$CVC_NAV_BUNDLE`` (a buildings.glb, or a directory holding one), else none (procedural
    massing). Python and the scene then make the same world."""
    env = os.environ.get("CVC_NAV_BUNDLE", "")
    if not env:
        return ""
    p = Path(env)
    p = p / "buildings.glb" if p.is_dir() else p
    return str(p) if p.is_file() else ""


def generate(params: dict, app=None) -> object:
    """libcvc's terrain generator (``pycvc.generate_world``) on ``params``: the generated_world
    node's world.* keys (seed, half, amp, water_level, buildings, trees, rocks, terrain, city, ...).
    ``app`` reads the building library when there is one (:func:`building_library`).
    """
    _require()
    clean = {k: v for k, v in params.items() if v not in (None, "")}
    for k in ("terrain", "city", "terrain_file"):
        if k in clean:
            clean[k] = str(clean[k])
    return pycvc.generate_world(clean, building_library(), app=app)


def world_from_generated(gen, *, cell=None) -> World:
    """A generated world (:func:`generate`): its terrain, buildings, hard ground and water."""
    terrain = gen.terrain()
    bounds = terrain.bounds2d()
    info = dict(gen.info())
    cell = cell or max(2.0, (bounds[2] - bounds[0]) / 600.0)
    occ = _rasterize(gen.world(), bounds, cell)
    return World("generated", occ, bounds, gen.height, gen.world(), {"source": "generated", **info})


def flat_world(unit: float) -> World:
    """An open square 40 car lengths (``unit``) across, walled at its edge."""
    half, cell = 20.0 * unit, 0.5 * unit
    n = int(round(2 * half / cell)) + 1
    occ = np.zeros((n, n), np.uint8)
    _border(occ)
    h, m = 1.2 * unit, half
    boxes = [(-m, -m, m, -m + cell, h), (-m, m - cell, m, m, h), (-m, -m, -m + cell, m, h),
             (m - cell, -m, m, m, h)]  # fmt: skip
    return World("flat", occ, (-half, -half, half, half), None, None, {"source": "flat"}, boxes)


def world_from_gym(gym) -> World:
    """The world a ``pycvc.WorldGym`` runs, as its navigator sees it: the grid of the document's
    world on (its occupancy and terrain height). Its physics is the gym's own, so ``geometry`` is
    None."""
    g = gym.grid()
    if g is None:
        raise RuntimeError("the gym's world has no grid yet")
    occ = np.ascontiguousarray(g["occupancy"], np.uint8)
    hgt = np.asarray(g["height"], float)
    w = World("gym", occ, tuple(g["bounds"]), None, None, {"source": "gym"})
    if np.any(hgt != 0.0):
        def height(x, y, _w=w, _h=hgt):  # the grid's sample nearest (x, y)
            r, c = _w.cells(x, y)
            return float(_h[r, c])

        w.height = height
    return w


# ── what the controller reads ──────────────────────────────────────────────────────────────────


@dataclass
class CarsState:
    """Every car at one tick, as (n,) arrays unless noted (SI: m, rad, m/s, m/s^2).

    ``x``, ``y`` (the rear axle's midpoint, the point the bicycle turns about), ``heading`` (0 along
    +x, counter-clockwise), ``speed`` (along the heading; negative backing up), ``steer`` (the front
    wheels' angle now; on a vehicle it lags the command at the steering's rate), ``vx``, ``vy`` (the
    velocity in the world), ``goal`` (n, 2), ``goal_dist``, ``reached`` (within ``reach`` m),
    ``ranges`` (n, R): the distance to anything blocked along each ray, at ``ray_angles`` (R,) off
    the heading, up to ``range_max``; ``collided``: touching something blocked or another car;
    the limits ``max_steer``, ``max_accel``, ``max_decel``, ``max_speed`` and the sizes ``length``,
    ``width``, ``wheelbase``, per car. ``t`` is the time (s), ``tick`` the ticks stepped, ``dt`` the
    tick, ``driving`` the plant, ``world`` the world's kind. ``nav`` holds the navigator's
    per-agent telemetry when it drives (alpha, beta, gamma, lam_soft, lam_hard, potential,
    clearance, ...)."""

    t: float
    tick: int
    dt: float
    x: np.ndarray
    y: np.ndarray
    heading: np.ndarray
    speed: np.ndarray
    steer: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    goal: np.ndarray
    goal_dist: np.ndarray
    reached: np.ndarray
    ranges: np.ndarray
    ray_angles: np.ndarray
    range_max: float
    collided: np.ndarray
    length: np.ndarray
    width: np.ndarray
    wheelbase: np.ndarray
    max_steer: np.ndarray
    max_accel: np.ndarray
    max_decel: np.ndarray
    max_speed: np.ndarray
    reach: float
    driving: str
    world: str
    nav: dict = field(default_factory=dict)

    @property
    def n(self) -> int:
        return int(self.x.shape[0])

    @property
    def pos(self) -> np.ndarray:
        """(n, 2) positions."""
        return np.stack([self.x, self.y], axis=1)

    @property
    def center(self) -> np.ndarray:
        """(n, 2) the bodies' centres (half a wheelbase ahead of the rear axle)."""
        h = 0.5 * self.wheelbase
        return np.stack([self.x + h * np.cos(self.heading), self.y + h * np.sin(self.heading)], 1)

    def copy(self) -> CarsState:
        out = CarsState(**{k: getattr(self, k) for k in self.__dataclass_fields__})
        for k, v in vars(out).items():
            if isinstance(v, np.ndarray):
                setattr(out, k, v.copy())
        return out


def ranges(world: World, x, y, heading, angles, range_max, offset=0.0):
    """(n, R) the distance along each ray (``angles`` off each heading) from ``offset`` m ahead of
    each (x, y) to the nearest blocked cell, ray-marched at half a cell, up to ``range_max``."""
    x, y, heading = (np.atleast_1d(np.asarray(a, float)) for a in (x, y, heading))
    cx, cy = x + offset * np.cos(heading), y + offset * np.sin(heading)
    step = 0.5 * world.cell
    d = np.arange(step, range_max + 1e-9, step)
    ang = heading[:, None] + angles[None, :]
    px = cx[:, None, None] + np.cos(ang)[:, :, None] * d[None, None, :]
    py = cy[:, None, None] + np.sin(ang)[:, :, None] * d[None, None, :]
    hit = world.occupied(px, py)
    first = np.where(hit.any(axis=2), hit.argmax(axis=2), len(d))
    return np.where(first < len(d), d[np.minimum(first, len(d) - 1)], range_max)


def _boxes_overlap(a, b):
    """Two oriented rectangles (cx, cy, cos, sin, half length, half width) overlap."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    for ax, ay in ((a[2], a[3]), (-a[3], a[2]), (b[2], b[3]), (-b[3], b[2])):
        ra = a[4] * abs(a[2] * ax + a[3] * ay) + a[5] * abs(-a[3] * ax + a[2] * ay)
        rb = b[4] * abs(b[2] * ax + b[3] * ay) + b[5] * abs(-b[3] * ax + b[2] * ay)
        if abs(dx * ax + dy * ay) > ra + rb:
            return False
    return True


def collisions(world: World, x, y, heading, length, width, wheelbase):
    """Each car touching something blocked (a point of its outline on a blocked cell) or another car
    (their boxes overlap)."""
    c, s = np.cos(heading), np.sin(heading)
    cx, cy = x + 0.5 * wheelbase * c, y + 0.5 * wheelbase * s
    hl, hw = 0.5 * length, 0.5 * width
    u = np.array([-1.0, -0.5, 0.0, 0.5, 1.0])
    px = np.concatenate([cx[:, None] + u * hl[:, None] * c[:, None] + sg * hw[:, None] * s[:, None]
                         for sg in (-1.0, 1.0)], 1)  # fmt: skip
    py = np.concatenate([cy[:, None] + u * hl[:, None] * s[:, None] - sg * hw[:, None] * c[:, None]
                         for sg in (-1.0, 1.0)], 1)  # fmt: skip
    out = world.occupied(px, py).any(axis=1)
    n = len(x)
    for i in range(n):
        for j in range(i + 1, n):
            if math.hypot(cx[i] - cx[j], cy[i] - cy[j]) > hl[i] + hl[j] + hw[i] + hw[j]:
                continue
            if _boxes_overlap((cx[i], cy[i], c[i], s[i], hl[i], hw[i]),
                              (cx[j], cy[j], c[j], s[j], hl[j], hw[j])):  # fmt: skip
                out[i] = out[j] = True
    return out


def state_from(world: World, x, y, heading, speed, goal, *, t=0.0, tick=0, dt=DT, steer=None,
               vx=None, vy=None, reach=7.0, driving="", length=None, width=None, wheelbase=None,
               max_steer=None, max_accel=None, max_decel=None, max_speed=None, rays=16,
               range_m=60.0, nav=None) -> CarsState:  # fmt: skip
    """A :class:`CarsState` for cars a host steps some other way (its own navigator, its own
    simulator), so :class:`CarsLog`, :func:`raster_arrays` and the panels take them as they take
    :class:`Cars`'. ``x``, ``y`` (the rear axle), ``heading``, ``speed`` are (n,) arrays and ``goal``
    is (n, 2), in metres and radians; the ranges and contacts are worked out on ``world`` as
    :class:`Cars` works them out. ``vx``, ``vy`` default to the speed along the heading, ``steer``
    to 0. A size or limit is one number for every car or an (n,) array (default: nav's bicycle, the
    Humvee preset's). ``nav`` is per-car telemetry, name -> (n,) array."""
    x, y, heading, speed = (np.atleast_1d(np.asarray(a, float)).copy() for a in (x, y, heading, speed))
    n = x.shape[0]
    goal = np.asarray(goal, float).reshape(n, 2).copy()

    def per_car(v, default):
        return np.broadcast_to(np.asarray(default if v is None else v, float), (n,)).copy()

    length = per_car(length, _BICYCLE["length"])
    width = per_car(width, _BICYCLE["width"])
    wheelbase = per_car(wheelbase, _BICYCLE["wheelbase"])
    angles = np.linspace(-math.pi, math.pi, int(rays), endpoint=False)
    d = goal - np.stack([x, y], 1)
    dist = np.hypot(d[:, 0], d[:, 1])
    return CarsState(
        t=float(t), tick=int(tick), dt=float(dt), x=x, y=y, heading=heading, speed=speed,
        steer=per_car(steer, 0.0),
        vx=per_car(vx, speed * np.cos(heading)), vy=per_car(vy, speed * np.sin(heading)),
        goal=goal, goal_dist=dist, reached=dist < reach,
        ranges=ranges(world, x, y, heading, angles, range_m, 0.5 * float(np.mean(wheelbase))),
        ray_angles=angles, range_max=float(range_m),
        collided=collisions(world, x, y, heading, length, width, wheelbase),
        length=length, width=width, wheelbase=wheelbase,
        max_steer=per_car(max_steer, _BICYCLE["max_steer"]),
        max_accel=per_car(max_accel, _BICYCLE["max_accel"]),
        max_decel=per_car(max_decel, _BICYCLE["max_decel"]),
        max_speed=per_car(max_speed, _BICYCLE["max_speed"]),
        reach=float(reach), driving=str(driving), world=world.kind,
        nav={k: np.asarray(v, float) for k, v in (nav or {}).items()},
    )  # fmt: skip


# ── the cars ───────────────────────────────────────────────────────────────────────────────────

# nav's bicycle, when no plant gives a size: the Humvee preset's body and limits.
_BICYCLE = {"length": 4.84, "width": 2.18, "wheelbase": 3.30, "rear": 0.77, "max_steer": 0.457,
            "max_accel": 2.5, "max_decel": 7.5, "max_speed": 24.6}  # fmt: skip


class Cars:
    """``n`` cars on ``world``, each with a random start and a goal, on the plant named ``vehicle``
    (``pycvc.plant_host``'s names: ``kinematic``, ``humvee``, ``roboracer``, a comma list cycled
    over the cars, ``dynamic[:presets]``, ``dynamic4w[:presets]``), driven by :meth:`step` (your
    ``(steer, accel)``) or, with ``navigator``, by libcvc's navigator (:meth:`step_navigator`).

    With ``app`` and ``prefix`` the settings live in that app's state tree: :meth:`pull` reads
    ``<prefix>.nav.*``, ``<prefix>.nav.vehicle.*`` and ``<prefix>.nav_plant.*`` (a new
    ``nav_plant.vehicle`` switches the plant, the cars keeping their poses) and :meth:`publish`
    writes them back with what is measured. ``nav_weights`` is a ``.cvcnav`` policy for the
    navigator ("" = libcvc's built-in one), and ``scale`` the frame it was trained in (normalized
    units per metre; nav's demos use 0.05). ``seed`` makes the starts and goals repeatable;
    ``starts`` and ``goals`` ((n, 2) world positions) set them instead, and the cars start facing
    their goals. ``rays`` and ``range_m`` shape the range readings."""

    def __init__(self, world: World, n=6, vehicle="humvee", *, seed=0, navigator=False,
                 nav_weights="", scale=SCALE, app=None, prefix="", rays=16, range_m=60.0,
                 starts=None, goals=None):  # fmt: skip
        _require()
        self.world, self.n = world, int(max(1, n))
        self.app, self.prefix = app, prefix
        self.navigator, self.nav_weights = bool(navigator), str(nav_weights or "")
        self.scale = float(scale)
        self.rng = np.random.default_rng(seed)
        self.ray_angles = np.linspace(-math.pi, math.pi, int(rays), endpoint=False)
        self.range_max = float(range_m)
        self.vehicle0 = vehicle
        self._sizes(vehicle)
        self.reach = max(3.0, 1.5 * float(np.max(self.length)))
        if starts is not None:
            self.starts = np.asarray(starts, float).reshape(self.n, 2).copy()
        else:
            self.starts = world.free_points(self.rng, self.n, 0.8 * float(np.max(self.length)),
                                            2.5 * float(np.max(self.length)))  # fmt: skip
        if goals is not None:
            self.goals = np.asarray(goals, float).reshape(self.n, 2).copy()
        else:
            self.goals = self._new_goals(self.starts)
        self._build(vehicle)
        self.t, self.tick = 0.0, 0
        self.last_command = (np.zeros(self.n), np.zeros(self.n))
        self.collisions = np.zeros(self.n, int)
        self.arrivals = 0
        self._state = self._read()

    def _sizes(self, vehicle):
        """Each car's body and limits: its preset's, else nav's bicycle's (the Humvee's)."""
        name = str(vehicle).split(":", 1)[-1] if ":" in str(vehicle) else str(vehicle)
        presets = [p.strip() for p in name.split(",") if p.strip() in ("humvee", "roboracer")]
        sizes = []
        for i in range(self.n):
            p = presets[i % len(presets)] if presets else None
            if p and hasattr(pycvc, "vehicle_desc"):
                d = getattr(pycvc.vehicle_desc, p)()
                hx, hy, _ = d.half_extents()
                axles = [d.axle(k) for k in range(d.axle_count())]
                rear = min(a["x"] for a in axles if not a["steered"])
                front = max(a["x"] for a in axles if a["steered"])
                sizes.append({"length": 2 * hx, "width": 2 * hy, "wheelbase": front - rear,
                              "rear": hx + rear, "max_steer": float(d.max_steer),
                              "max_accel": float(d.max_accel), "max_decel": float(d.max_decel),
                              "max_speed": float(d.max_speed), "preset": p})  # fmt: skip
            else:
                sizes.append({**_BICYCLE, "preset": None})
        for k in ("length", "width", "wheelbase", "rear", "max_steer", "max_accel", "max_decel",
                  "max_speed"):  # fmt: skip
            setattr(self, k, np.array([s[k] for s in sizes], float))
        #: Each car's vehicle preset (humvee, roboracer), or None on nav's bicycle (a Humvee's body).
        self.preset = [s["preset"] for s in sizes]

    def _build(self, vehicle):
        w = self.world
        rows, cols = w.occ.shape
        self.cx, self.cy = w.center
        S = self.scale
        on = ((self.starts - [self.cx, self.cy]) * S).astype(np.float32)
        goal = ((self.goals - [self.cx, self.cy]) * S).astype(np.float32)
        L = float(np.mean(self.wheelbase))
        vmax = float(np.max(self.max_speed))
        # Footprint radius and barrier as nav's Humvee demos have them, scaled to the car.
        k = float(np.max(self.length)) / 4.84
        self.sim = pycvc.nav_sim_world_create(
            w.occ, w.occ, self.nav_weights, on, goal, np.ones((self.n, 3), np.float32), rows, cols,
            *w.bounds, self.cx, self.cy, S, 60.0 * k, 120, 2 * math.pi, 1.2 * k * S, 3.0 * k * S, DT,
            vmax * S, L * S, float(np.min(self.max_steer)), 0.8 * float(np.min(self.max_accel)) * S,
            0.5 * 9.80665 * S, 0.8, NSUB, 1, self.reach * S, 4, 1, 2.2, -1.4, 8.0, 1, 0.5, 0.15, 4.0)  # fmt: skip
        d = self.goals - self.starts
        for i, h in enumerate(np.arctan2(d[:, 1], d[:, 0])):
            pycvc.nav_sim_world_set_pose(self.sim, i, float(on[i, 0]), float(on[i, 1]), float(h))
        if self.navigator:
            pycvc.nav_sim_world_set_capture_drive_telemetry(self.sim, 1)
        self.host = pycvc.plant_host(fixed_dt=0.01)
        if w.geometry is not None:
            self.host.set_geometry(w.geometry)
        self.driving = self.host.use(self.sim, vehicle)
        if self.app is not None:  # the tree's settings apply (unset ones are seeded), then publish
            self.pull()
            self.publish()

    # ── the state tree ──
    def pull(self):
        """Read the settings from the app's state tree (between ticks). A new nav_plant.vehicle
        switches the plant: the cars keep their poses and start at rest."""
        if self.app is None:
            return
        pycvc.nav_sim_world_pull(self.sim, self.app, self.prefix)
        before = self.host.vehicle()
        self.host.pull(self.app, self.prefix, self.sim)
        if self.host.vehicle() != before:
            self._sizes(self.host.vehicle())
        self.driving = self.host.driving()

    def publish(self):
        if self.app is None:
            return
        pycvc.nav_sim_world_publish(self.sim, self.app, self.prefix)
        p = f"{self.prefix}.nav_plant" if self.prefix else "nav_plant"
        if not pycvc.state_has(self.app, f"{p}.vehicle"):
            pycvc.state_set(self.app, f"{p}.vehicle", self.host.vehicle())
        self.host.pull(self.app, self.prefix, self.sim)

    # ── goals ──
    def _new_goals(self, near):
        return self.world.free_points(self.rng, self.n, 0.8 * float(np.max(self.length)),
                                      4.0 * float(np.max(self.length)), avoid=near,
                                      avoid_dist=0.3 * self.world.span)  # fmt: skip

    def new_goals(self, which=None):
        """Give every car (or those in ``which``, a bool mask or indices) a new goal. A car at its
        goal that gets a new one counts in :attr:`arrivals`."""
        fresh = self._new_goals(self._state.pos)
        mask = np.ones(self.n, bool) if which is None else np.zeros(self.n, bool)
        if which is not None:
            mask[which] = True
        self.arrivals += int((mask & self._state.reached).sum())
        self.goals = np.where(mask[:, None], fresh, self.goals)
        for i in np.flatnonzero(mask):
            gx, gy = self.goals[i]
            pycvc.nav_sim_world_retarget(
                self.sim, int(i), (gx - self.cx) * self.scale, (gy - self.cy) * self.scale
            )
        self._state = self._read()
        return self._state

    # ── stepping ──
    def step(self, steer, accel) -> CarsState:
        """One tick: each car's plant takes ``steer`` (rad, + left; held to its lock) and ``accel``
        (m/s^2 along the heading; held to [-max_decel, max_accel], and the speed to its top speed).
        A NaN leaves that car to the navigator for the tick. Returns the new state."""
        steer = np.asarray(steer, float).reshape(self.n)
        accel = np.asarray(accel, float).reshape(self.n)
        steer = np.where(np.isnan(steer), np.nan, np.clip(steer, -self.max_steer, self.max_steer))
        accel = np.where(np.isnan(accel), np.nan, np.clip(accel, -self.max_decel, self.max_accel))
        pycvc.nav_sim_world_override(self.sim, steer, accel * self.scale)
        pycvc.nav_sim_world_step(self.sim)
        self.last_command = (steer, accel)
        self._steer_cmd = np.where(np.isnan(steer), 0.0, steer)
        return self._advance()

    def step_navigator(self) -> CarsState:
        """One tick of libcvc's navigator driving every car to its goal."""
        pycvc.nav_sim_world_step(self.sim)
        self.last_command = (np.full(self.n, np.nan), np.full(self.n, np.nan))
        return self._advance()

    def _advance(self):
        self.t += DT
        self.tick += 1
        self._state = self._read()
        self.collisions += self._state.collided
        return self._state

    def state(self) -> CarsState:
        return self._state

    # ── reading ──
    def _read(self) -> CarsState:
        pos, head, speed = pycvc.nav_sim_world_snapshot(self.sim)[:3]
        x, y = pos[:, 0].astype(float), pos[:, 1].astype(float)
        th, sp = head.astype(float), speed.astype(float)
        steer = getattr(self, "_steer_cmd", np.zeros(self.n)).copy()
        vx, vy = sp * np.cos(th), sp * np.sin(th)
        kind = self.host.kind()
        if kind != "kinematic":
            for i in range(self.n):
                st = self.host.state(i)
                if st and "steer" in st:
                    steer[i] = float(st["steer"])
                vs = self.host.vehicle_state(i) if kind == "jolt" else None
                if vs:
                    vx[i], vy[i] = vs["linear_velocity"][0], vs["linear_velocity"][1]
                b = self.host.body(i) if kind == "dynamic" else None
                if b:
                    c, s = math.cos(b["psi"]), math.sin(b["psi"])
                    vx[i], vy[i] = b["vx"] * c - b["vy"] * s, b["vx"] * s + b["vy"] * c
                    steer[i] = float(b["delta"])
        d = self.goals - np.stack([x, y], 1)
        dist = np.hypot(d[:, 0], d[:, 1])
        nav = {}
        if self.navigator:
            tel = pycvc.nav_sim_world_drive_telemetry(self.sim)
            if tel:
                nav = {k: np.asarray(v, float) for k, v in tel.items()
                       if k in ("alpha", "beta", "gamma", "lam_soft", "lam_hard", "potential",
                                "clearance", "curvature", "v_risk", "mu")}  # fmt: skip
        return CarsState(
            t=self.t, tick=self.tick, dt=DT, x=x, y=y, heading=th, speed=sp, steer=steer, vx=vx,
            vy=vy, goal=self.goals.copy(), goal_dist=dist, reached=dist < self.reach,
            ranges=ranges(self.world, x, y, th, self.ray_angles, self.range_max,
                          0.5 * float(np.mean(self.wheelbase))),
            ray_angles=self.ray_angles.copy(), range_max=self.range_max,
            collided=collisions(self.world, x, y, th, self.length, self.width, self.wheelbase),
            length=self.length, width=self.width, wheelbase=self.wheelbase,
            max_steer=self.max_steer, max_accel=self.max_accel, max_decel=self.max_decel,
            max_speed=self.max_speed, reach=self.reach, driving=self.driving, world=self.world.kind,
            nav=nav,
        )  # fmt: skip


class GymCars:
    """``n`` cars in a world document, run by ``pycvc.WorldGym`` (headless): the document's world,
    its ``world: vehicles:`` archetypes as Jolt vehicles in its own physics world, and its
    ``spawns:`` (or a free point of its grid) for the starts. The same interface as :class:`Cars`
    for a controller and the template's loop: :meth:`state`, :meth:`step`, :meth:`step_navigator`,
    :meth:`new_goals`, ``world`` (:func:`world_from_gym`, for :func:`rollout` and the rasters).

    ``vehicle`` names the archetype (None: the document's first), or the plant as :class:`Cars`
    names it: ``kinematic``, ``jolt``, ``dynamic`` or ``dynamic4w``, with ``:<archetype>`` after
    it to name the archetype too (``dynamic:roboracer``). The archetype must allow ``nav@1`` for
    :meth:`step_navigator` to drive. ``seed`` makes the starts and goals repeatable. ``gym_args``
    go to ``pycvc.WorldGym`` (``prefix``, ``world``, ``set``, ``ready_timeout``).

    What differs from :class:`Cars`: the plant runs in the document's world (a Jolt vehicle in
    its physics world, the dynamic models in a world of their own holding its colliders, nav's
    bicycle on its grid), the navigator's telemetry (``CarsState.nav``) is empty, and
    ``collided`` is the gym's: a contact with the world's bodies other than its ground, or with
    another vehicle, over the tick."""

    def __init__(self, document, n=6, vehicle=None, *, seed=0, navigator=False, rays=16,
                 range_m=60.0, **gym_args):  # fmt: skip
        _require()
        self.n = int(max(1, n))
        agent = {"controller": "nav@1"}
        name = str(vehicle or "")
        plant, _, rest = name.partition(":")
        if plant in GYM_PLANTS:
            agent["plant"] = plant
            if rest:
                agent["archetype"] = rest.split(",")[0]
        elif name:
            agent["archetype"] = name
        self.gym = pycvc.WorldGym(str(document), agents=[dict(agent) for _ in range(self.n)],
                                  wander=False, max_steps=0, rays=int(rays),
                                  range_m=float(range_m), **gym_args)  # fmt: skip
        self.navigator = bool(navigator)
        self.rng = np.random.default_rng(seed)
        self.world = world_from_gym(self.gym)
        self.driving = agent.get("plant", "")
        self.arrivals = 0
        self._reset(seed)

    def _reset(self, seed):
        o = self.gym.reset(seed=int(seed))
        if not self.driving:  # the archetype's own plant
            self.driving = (o["plant"][0] if o["plant"] else "") or "jolt"
        for k in ("length", "width", "wheelbase", "max_steer", "max_accel", "max_decel",
                  "max_speed"):  # fmt: skip
            setattr(self, k, np.asarray(o[k], float).copy())
        self.preset = [None] * self.n
        self.reach = max(3.0, 1.5 * float(np.max(self.length)))
        self.t, self.tick = 0.0, 0
        self.last_command = (np.zeros(self.n), np.zeros(self.n))
        self.collisions = np.zeros(self.n, int)
        self._state = self._read(o)

    def reset(self, seed=0) -> CarsState:
        """A new episode: the document's world as it built it, the cars spawned again."""
        self.rng = np.random.default_rng(seed)
        self.arrivals = 0
        self._reset(seed)
        return self._state

    # The template's loop calls these; the gym's settings are its document's.
    def pull(self):
        pass

    def publish(self):
        pass

    @property
    def goals(self) -> np.ndarray:
        return self._state.goal.copy()

    def new_goals(self, which=None):
        """Every car (or those in ``which``, a bool mask or indices) gets a new goal, a free point
        of the grid away from where it is. A car at its goal that gets one counts in
        :attr:`arrivals`."""
        s = self._state
        fresh = self.world.free_points(self.rng, self.n, 0.8 * float(np.max(self.length)),
                                       4.0 * float(np.max(self.length)), avoid=s.pos,
                                       avoid_dist=0.3 * self.world.span)  # fmt: skip
        mask = np.ones(self.n, bool) if which is None else np.zeros(self.n, bool)
        if which is not None:
            mask[which] = True
        self.arrivals += int((mask & s.reached).sum())
        ids = self.gym.observe()["id"]
        for i in np.flatnonzero(mask):
            self.gym.set_goal(int(ids[i]), float(fresh[i, 0]), float(fresh[i, 1]))
        self._state = self._read(self.gym.observe())
        return self._state

    def step(self, steer, accel) -> CarsState:
        """One tick: each car's ``steer`` (rad, + left) and ``accel`` (m/s^2), held to its lock and
        envelope; a NaN leaves that car to its controller (nav@1) for the tick."""
        steer = np.asarray(steer, float).reshape(self.n)
        accel = np.asarray(accel, float).reshape(self.n)
        steer = np.where(np.isnan(steer), np.nan, np.clip(steer, -self.max_steer, self.max_steer))
        accel = np.where(np.isnan(accel), np.nan, np.clip(accel, -self.max_decel, self.max_accel))
        o = self.gym.step(steer, accel)[0]
        self.last_command = (steer, accel)
        return self._advance(o)

    def step_navigator(self) -> CarsState:
        """One tick of the document's controllers (nav@1) driving every car to its goal."""
        nan = np.full(self.n, np.nan)
        o = self.gym.step(nan, nan)[0]
        self.last_command = (nan, nan.copy())
        return self._advance(o)

    def _advance(self, o):
        self.t = float(o["time"])
        self.tick = int(o["step"])
        self._state = self._read(o)
        self.collisions += self._state.collided
        return self._state

    def state(self) -> CarsState:
        return self._state

    def _read(self, o) -> CarsState:
        th = np.asarray(o["yaw"], float)
        wb = self.wheelbase
        # The gym gives the chassis's centre; a CarsState's (x, y) is the rear axle.
        x = np.asarray(o["x"], float) - 0.5 * wb * np.cos(th)
        y = np.asarray(o["y"], float) - 0.5 * wb * np.sin(th)
        goal = np.stack([np.asarray(o["goal_x"], float), np.asarray(o["goal_y"], float)], 1)
        dist = np.hypot(goal[:, 0] - x, goal[:, 1] - y)
        return CarsState(
            t=float(o["time"]), tick=int(o["step"]), dt=float(self.gym.dt()), x=x, y=y,
            heading=th, speed=np.asarray(o["speed"], float), steer=np.asarray(o["steer"], float),
            vx=np.asarray(o["vx"], float), vy=np.asarray(o["vy"], float), goal=goal,
            goal_dist=dist, reached=dist < self.reach,
            ranges=np.asarray(o["ranges"], float), ray_angles=np.asarray(o["ray_angles"], float),
            range_max=float(o["range_m"]), collided=np.asarray(o["collided"], bool).copy(),
            length=self.length, width=self.width, wheelbase=self.wheelbase,
            max_steer=self.max_steer, max_accel=self.max_accel, max_decel=self.max_decel,
            max_speed=self.max_speed, reach=self.reach, driving=self.driving,
            world=self.world.kind, nav={},
        )  # fmt: skip


# ── a controller's own prediction ──────────────────────────────────────────────────────────────


def rollout(controller, s: CarsState, p: dict, world: World, steps=40, every=1):
    """Each car's path over the next ``steps`` ticks if ``controller`` kept driving it on nav's
    bicycle (no slip, instant steering, no collisions), the cars moving together: (n, steps // every
    + 1, 2). What a controller plans, drawn and logged; not what the plant will do."""
    q = s.copy()
    out = [q.pos.copy()]
    h = DT / NSUB
    for k in range(steps):
        r = controller(q, p)
        steer = np.clip(np.nan_to_num(np.asarray(r[0], float)), -q.max_steer, q.max_steer)
        accel = np.clip(np.nan_to_num(np.asarray(r[1], float)), -q.max_decel, q.max_accel)
        t = np.tan(steer)
        for _ in range(NSUB):
            q.speed = np.clip(q.speed + h * accel, -0.25 * q.max_speed, q.max_speed)
            q.heading = q.heading + h * (q.speed / q.wheelbase) * t
            q.x = q.x + h * q.speed * np.cos(q.heading)
            q.y = q.y + h * q.speed * np.sin(q.heading)
        q.steer = steer
        q.vx, q.vy = q.speed * np.cos(q.heading), q.speed * np.sin(q.heading)
        d = q.goal - q.pos
        q.goal_dist = np.hypot(d[:, 0], d[:, 1])
        q.reached = q.goal_dist < q.reach
        q.ranges = ranges(
            world, q.x, q.y, q.heading, q.ray_angles, q.range_max, 0.5 * float(np.mean(q.wheelbase))
        )
        q.t += DT
        if (k + 1) % every == 0:
            out.append(q.pos.copy())
    return np.stack(out, axis=1)


# ── rasters ────────────────────────────────────────────────────────────────────────────────────


def local_grid(x, y, heading, size_m, n):
    """World (X, Y) of an n x n grid ``size_m`` across centred on (x, y), heading up: row 0 is the
    far edge ahead, column 0 the left."""
    u = (np.arange(n) + 0.5) / n - 0.5
    fwd, left = np.meshgrid(-u * size_m, -u * size_m, indexing="ij")  # row: ahead -> behind
    c, s = math.cos(heading), math.sin(heading)
    X = x + fwd * c - left * s
    Y = y + fwd * s + left * c
    return X, Y


def local_raster(world: World, x, y, heading, size_m=60.0, n=96):
    """The occupancy (bool) and signed distance (m) around (x, y), heading up (:func:`local_grid`)."""
    X, Y = local_grid(x, y, heading, size_m, n)
    r, c = world.cells(X, Y)
    return {"occ": world.occupied(X, Y), "sdf": world.sdf[r, c]}


def colorize(values, lo=None, hi=None, occ=None):
    """A field as an RGB picture: dark blue (low) through pale (high), blocked cells charcoal."""
    v = np.asarray(values, float)
    lo = np.nanmin(v) if lo is None else lo
    hi = np.nanmax(v) if hi is None else hi
    t = np.clip((v - lo) / max(hi - lo, 1e-9), 0.0, 1.0)
    rgb = np.stack([40 + 200 * t, 70 + 170 * t, 140 + 90 * t], axis=-1)
    if occ is not None:
        rgb[np.asarray(occ, bool)] = (52, 54, 60)
    return np.nan_to_num(rgb).astype(np.uint8)


def draw_path(img, pts, x, y, heading, size_m, color):
    """Mark world points ``pts`` (k, 2) on a :func:`local_grid` picture."""
    n = img.shape[0]
    c, s = math.cos(heading), math.sin(heading)
    for px, py in pts:
        dx, dy = px - x, py - y
        fwd, left = dx * c + dy * s, -dx * s + dy * c
        r = int((0.5 - fwd / size_m) * n)
        k = int((0.5 - left / size_m) * n)
        if 0 <= r < n and 0 <= k < n:
            img[max(r - 1, 0) : r + 2, max(k - 1, 0) : k + 2] = color
    return img


def raster_arrays(world: World, s: CarsState, k: int, size_m: float, n=96, *, belief=None,
                  cost=None) -> dict:  # fmt: skip
    """Car ``k``'s surroundings, heading up (:func:`local_grid`), as the log keeps them: ``sdf``
    (float32, m) and ``occ`` (uint8); with ``belief`` (an occupancy grid on ``world``'s cells, as an
    agent believes it: nonzero blocked, or a float probability of blocked for an agent that has
    not seen everything) also ``belief`` sampled there (uint8, or float32 for a probability); with
    ``cost`` (``cost(X, Y) ->
    array``, world coordinates) also ``cost`` (float32)."""
    loc = local_raster(world, s.x[k], s.y[k], s.heading[k], size_m, n)
    out = {"sdf": loc["sdf"].astype(np.float32), "occ": loc["occ"].astype(np.uint8)}
    X, Y = local_grid(s.x[k], s.y[k], s.heading[k], size_m, n)
    if belief is not None:
        r, c = world.cells(X, Y)
        b = np.asarray(belief)[r, c]
        out["belief"] = b.astype(np.float32) if b.dtype.kind == "f" else (b != 0).astype(np.uint8)
    if cost is not None:
        out["cost"] = np.asarray(cost(X, Y), np.float32)
    return out


def raster_pictures(r: dict, s: CarsState, k: int, size_m: float, *, path=None) -> dict:
    """:func:`raster_arrays`' rasters as the cars_rasters panel's pictures: ``raster.sdf``, the
    signed distance with car ``k``'s ``path`` ((m, 2) world points, yellow), its goal (green) and
    the other cars (red), and ``raster.cost``, the cost (or else the belief) when there is one."""
    img = colorize(np.clip(r["sdf"], -2.0, 0.25 * size_m), occ=r["occ"])
    if path is not None:
        draw_path(img, path, s.x[k], s.y[k], s.heading[k], size_m, (250, 200, 40))
    for j in range(s.n):
        if j != k:
            draw_path(img, [s.pos[j]], s.x[k], s.y[k], s.heading[k], size_m, (230, 90, 70))
    draw_path(img, [s.goal[k]], s.x[k], s.y[k], s.heading[k], size_m, (90, 230, 120))
    out = {"raster.sdf": img}
    if "cost" in r:
        out["raster.cost"] = colorize(r["cost"], occ=r["occ"])
    elif "belief" in r:
        out["raster.cost"] = colorize(r["belief"].astype(float), 0, 1)
    return out


def chosen(host, n: int) -> int:
    """The chosen car (``view.car``), held to the cars there are."""
    return min(max(0, host.get_int("view.car", 0)), int(n) - 1)


def raster_size(host, length: float) -> float:
    """The pictures' width (m): ``view.raster_m``, else twelve car lengths."""
    return host.get_float("view.raster_m", 0.0) or 12.0 * float(length)


# ── settings ───────────────────────────────────────────────────────────────────────────────────


def settings(app, prefix, roots=("nav", "nav_plant")) -> list[dict]:
    """Every setting under ``<prefix>.<root>`` with what its key says it is: path, value, comment,
    access (knob / telemetry / build), unit, min, max."""
    out = []

    def walk(path):
        for child in pycvc.state_children(app, path) or []:
            p = f"{path}.{child}"
            kids = pycvc.state_children(app, p) or []
            if kids:
                walk(p)
            prop = {}
            for key in pycvc.state_property_keys(app, p) or []:
                prop[key] = pycvc.state_property(app, p, key)
            if not kids or prop:
                out.append({"path": p[len(prefix) + 1 :] if prefix else p,
                            "value": pycvc.state_get(app, p), "comment": pycvc.state_comment(app, p),
                            **prop})  # fmt: skip

    for root in roots:
        base = f"{prefix}.{root}" if prefix else root
        if pycvc.state_has(app, base):
            walk(base)
    return out


# ── logging ────────────────────────────────────────────────────────────────────────────────────

#: What :class:`CarsLog` records each tick, per car.
LOG_FIELDS = ("x", "y", "heading", "speed", "steer", "vx", "vy", "goal_x", "goal_y", "goal_dist",
              "reached", "collided", "cmd_steer", "cmd_accel")  # fmt: skip


class CarsLog:
    """Every tick's state and command per car (:meth:`record`), the rasters and coefficients the
    panels show (:meth:`raster`, :meth:`coeffs`), and the run's settings and world (:attr:`meta`).
    :meth:`save` writes ``.npz`` (arrays (ticks, cars) named as :data:`LOG_FIELDS`, ``t``,
    ``raster_<name>`` (k, h, w) with ``raster_t``, ``coef_<name>`` (ticks, cars), and ``meta``, a
    JSON string) or ``.csv`` (one row per car per tick; the rasters and meta beside it in
    ``<name>.npz`` when there are any)."""

    def __init__(self):
        self.t, self.rows = [], {k: [] for k in LOG_FIELDS}
        self.rasters, self.raster_t = {}, []
        self.coef = {}
        self.meta = {}

    def record(self, s: CarsState, command=None):
        steer, accel = command if command is not None else (np.full(s.n, np.nan),) * 2
        vals = {"x": s.x, "y": s.y, "heading": s.heading, "speed": s.speed, "steer": s.steer,
                "vx": s.vx, "vy": s.vy, "goal_x": s.goal[:, 0], "goal_y": s.goal[:, 1],
                "goal_dist": s.goal_dist, "reached": s.reached, "collided": s.collided,
                "cmd_steer": steer, "cmd_accel": accel}  # fmt: skip
        self.t.append(s.t)
        for k in LOG_FIELDS:
            self.rows[k].append(np.asarray(vals[k], float).copy())

    def coeffs(self, values: dict, n: int):
        """A tick's coefficients: name -> (n,) array (a missing name this tick is NaN)."""
        k = len(self.t)
        for name, v in values.items():
            col = self.coef.setdefault(name, [])
            while len(col) < k - 1:
                col.append(np.full(n, np.nan))
            col.append(np.asarray(v, float).reshape(-1)[:n].copy())

    def raster(self, t, **images):
        self.raster_t.append(t)
        for name, img in images.items():
            self.rasters.setdefault(name, []).append(np.asarray(img).copy())

    def __len__(self):
        return len(self.t)

    def arrays(self) -> dict:
        out = {k: np.array(v) for k, v in self.rows.items()}
        out["t"] = np.array(self.t)
        n = out["x"].shape[1] if len(self.t) else 0
        for name, col in self.coef.items():
            col = col + [np.full(n, np.nan)] * (len(self.t) - len(col))
            out[f"coef_{name}"] = np.array(col)
        for name, imgs in self.rasters.items():
            out[f"raster_{name}"] = np.array(imgs)
        if self.raster_t:
            out["raster_t"] = np.array(self.raster_t)
        out["meta"] = np.array(json.dumps(self.meta, default=str))
        return out

    def save(self, path) -> str:
        path = str(path)
        if path.lower().endswith(".csv"):
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["tick", "t", "car", *LOG_FIELDS])
                for k, t in enumerate(self.t):
                    for i in range(len(self.rows["x"][k])):
                        w.writerow(
                            [k, f"{t:.4f}", i, *(f"{self.rows[f][k][i]:.6g}" for f in LOG_FIELDS)]
                        )
            extra = {
                k: v
                for k, v in self.arrays().items()
                if k.startswith(("raster_", "coef_")) or k == "meta"
            }
            np.savez_compressed(path[:-4] + ".npz", **extra)
        else:
            np.savez_compressed(path, **self.arrays())
        return path


# ── the host: the document, its panels and the loop ───────────────────────────────────────────

COLORS = [(0.90, 0.33, 0.20), (0.18, 0.55, 0.90), (0.95, 0.75, 0.15), (0.35, 0.75, 0.35),
          (0.70, 0.40, 0.85), (0.20, 0.80, 0.80), (0.95, 0.55, 0.70), (0.55, 0.55, 0.55)]  # fmt: skip

HERE = Path(__file__).resolve().parent

#: The directory holding cars_panels.ari, the template's panels as units: a host passes it as a
#: component path (``Host(component_paths=[COMPONENTS])``) and its document imports
#: ``cvc://cars_panels.ari`` and includes them (cars_settings, cars_world, cars_world_choice,
#: cars_coefficients, cars_rasters).
COMPONENTS = str(HERE)


def template_files() -> tuple[Path, Path]:
    """The template's two files in this package: cars_template.py and cars.ari."""
    return HERE / "cars_template.py", HERE / "cars.ari"


def copy_template(dest, force=False) -> int:
    """Write the template (cars.py and cars.ari) into ``dest`` to edit."""
    dest = Path(dest)
    dest.mkdir(parents=True, exist_ok=True)
    src_py, src_ari = template_files()
    for src, name in ((src_py, "cars.py"), (src_ari, "cars.ari")):
        out = dest / name
        if out.exists() and not force:
            print(f"cars: {out} exists (--force replaces it)", file=sys.stderr)
            return 1
        shutil.copyfile(src, out)
    print(f"cars: wrote {dest / 'cars.py'} and {dest / 'cars.ari'}\n"
          f"  edit controller() in cars.py, then:  python {dest / 'cars.py'}")  # fmt: skip
    return 0


def _box(x0, y0, x1, y1, z0, z1):
    v = [
        x0,
        y0,
        z0,
        x1,
        y0,
        z0,
        x1,
        y1,
        z0,
        x0,
        y1,
        z0,
        x0,
        y0,
        z1,
        x1,
        y0,
        z1,
        x1,
        y1,
        z1,
        x0,
        y1,
        z1,
    ]
    t = [4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7]
    return [float(a) for a in v], t


def _merge(meshes):
    verts, tris = [], []
    for v, t in meshes:
        b = len(verts) // 3
        verts += v
        tris += [b + k for k in t]
    return verts, tris


def _car_mesh(length, width, height):
    """A body and a cabin set back from its nose, +X forward, base at z = 0, centred."""
    hl, hw = 0.5 * length, 0.5 * width
    return _merge([_box(-hl, -hw, hl, hw, 0.12 * height, 0.55 * height),
                   _box(-0.6 * hl, -0.85 * hw, 0.25 * hl, 0.85 * hw, 0.55 * height, height)])  # fmt: skip


def add_args(p):
    """Add the flags :func:`run` shares with any host built on these pieces to the
    ``argparse`` parser ``p``: ``--world``, ``--uri``, ``--data-path`` (the world), ``--set``,
    ``--params`` (settings, :func:`apply_settings`), ``--log``, ``--raster-every`` and
    ``--min-arrivals`` (the headless run, :func:`save_log` and :func:`exit_code`)."""
    p.add_argument(
        "--world", choices=WORLDS, default=None, help="data (default), generated or flat"
    )
    p.add_argument("--uri", default=None,
                   help="the data world: a cvcpkg://ORG/NAME package, an https archive or a local "
                        "bundle directory (default: the document's data: block)")  # fmt: skip
    p.add_argument(
        "--data-path", default=None, help="the part of the package to keep (a cvcpkg uri)"
    )
    p.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                   help="a state-tree setting, relative to the document (nav.vehicle.vmax=12, "
                        "world.seed=3, ctl.gain=2); repeatable")  # fmt: skip
    p.add_argument("--params", default=None, help="a JSON file of settings, {key: value}")
    p.add_argument("--log", default=None, help="write every tick (.npz, or .csv + .npz)")
    p.add_argument("--raster-every", type=int, default=10, help="log the rasters every N ticks")
    p.add_argument("--min-arrivals", type=int, default=None,
                   help="exit 3 when fewer goals were reached (for scripted experiments)")  # fmt: skip
    return p


def _parse(argv, doc_default):
    from pycvc_gl.ariadne import main_args

    host_ns = main_args(argv)
    p = argparse.ArgumentParser(
        prog="cars.py", description="N Ackermann cars driven by your controller"
    )
    p.add_argument("--cars", type=int, default=None, help="how many cars (default: the document's)")
    p.add_argument("--vehicle", default=None,
                   help="the plant: kinematic, humvee, roboracer, a comma list, dynamic[:presets] "
                        "(sets nav_plant.vehicle)")  # fmt: skip
    add_args(p)
    p.add_argument("--controller", choices=("yours", "navigator"), default=None)
    p.add_argument("--nav-weights", default="", help="a .cvcnav policy for the navigator")
    p.add_argument("--nav-scale", type=float, default=SCALE,
                   help="the navigator's normalized units per metre: the frame its policy was "
                        "trained in (default 0.05)")  # fmt: skip
    p.add_argument("--seed", type=int, default=0, help="the starts and goals")
    p.add_argument("--gym", default=None, metavar="DOC",
                   help="drive the cars in a world document (pycvc.WorldGym, headless): its world, "
                        "its vehicles (--vehicle names an archetype) and its spawn points")  # fmt: skip
    p.add_argument("--doc", default=doc_default, help=argparse.SUPPRESS)
    ns = p.parse_args(host_ns.rest)
    for k, v in vars(host_ns).items():
        if k != "rest":
            setattr(ns, k, v)
    return ns


def apply_settings(host, args, flags=None) -> dict:
    """Set what :func:`add_args`' flags ask for over the document's ``init:``: the ``--params``
    file's keys, then each ``--set``, then ``flags`` ({key: value}; a None value is skipped), and
    ``--world`` (``run.world``). Then ``--uri`` points the document's ``data.city`` at that world
    (``--data-path`` the part of it to keep). Keys are relative to the document. Returns the keys
    set."""
    overrides = {}
    if getattr(args, "params", None):
        overrides.update(json.loads(Path(args.params).read_text()))
    for kv in getattr(args, "set", None) or []:
        k, _, v = kv.partition("=")
        overrides[k.strip()] = v.strip()
    for key, value in (flags or {}).items():
        if value is not None:
            overrides[key] = value
    if getattr(args, "world", None) is not None:
        overrides["run.world"] = args.world
    for k, v in overrides.items():
        host.set(k, v)
    if getattr(args, "uri", None):
        host.set("run.data_path", args.data_path or "")
        load_data_world(host, args.uri, args.data_path)
    return overrides


def save_log(log, host, path, **meta) -> str:
    """Save ``log`` to ``path`` with every setting (:func:`settings`: ``nav.*``, ``nav_plant.*``)
    and the generator's ``world.*`` keys in its meta, and ``meta`` besides. Returns the path."""
    log.meta["settings"] = {d["path"]: d["value"] for d in settings(host.app, host.prefix)}
    log.meta["world_keys"] = {k: host.get(f"world.{k}") for k in WORLD_KEYS}
    log.meta.update(meta)
    return log.save(path)


def exit_code(code=0, *, error=None, arrivals=0, min_arrivals=None, who="cars") -> int:
    """The run's exit code: 1 when it stopped on an ``error`` (a controller or world that failed),
    3 when ``min_arrivals`` was asked for and fewer goals were reached, else ``code`` (the
    document's)."""
    if error:
        print(f"{who}: stopped: {error}", file=sys.stderr)
        return 1
    if min_arrivals is not None and arrivals < min_arrivals:
        print(f"{who}: {arrivals} arrivals, fewer than --min-arrivals {min_arrivals}",
              file=sys.stderr)  # fmt: skip
        return 3
    return code


def run(controller, params: dict, *, cost=None, doc=None, argv=None) -> int:
    """Run the template: ``controller(s, p) -> (steer, accel)`` or ``(steer, accel, info)`` (info:
    ``{"coeffs": {name: (n,) array}}`` to plot and log), ``params`` its parameters (``ctl.<name>``
    in the document), ``cost(s, p, i, X, Y) -> array`` (optional) a field to draw around the chosen
    car, ``doc`` the .ari. Returns the exit code: 0, 1 when the controller or the world failed, 3
    when ``--min-arrivals`` was not met."""
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--copy" in argv:
        k = argv.index("--copy")
        if k + 1 >= len(argv):
            print("cars: --copy DIR", file=sys.stderr)
            return 2
        return copy_template(argv[k + 1], force="--force" in argv)
    _require()
    doc = doc or str(template_files()[1])
    args = _parse(argv, doc)
    from pycvc_gl.ariadne import Host

    host = Host(mode=args.mode, name="cars", width=args.width, height=args.height,
                component_paths=[COMPONENTS])
    pycvc.data_allow_network(host.app, ["https://cvcpkg.org"])
    host.load(args.doc)
    pre = host.prefix
    # The settings: the document's init:, then a params file, then --set, then the flags.
    for k, v in PARAMS_DEFAULTS(params).items():
        if host.get(f"ctl.{k}") in (None, ""):
            host.set(f"ctl.{k}", v)
    apply_settings(host, args, {"run.cars": args.cars, "nav_plant.vehicle": args.vehicle,
                                "run.controller": args.controller})  # fmt: skip

    drawn = host.mode != "headless"
    if args.gym and drawn:
        print("cars: --gym runs headless (open the world document in arigl to watch it)",
              file=sys.stderr)  # fmt: skip
        host.close()
        return 2
    log = CarsLog() if args.log else None
    st = {"cars": None, "speed": [], "gap": [], "coef": {}, "camera": None, "nodes": [], "error": None,
          "paths": None, "world": None, "frame": 0}  # fmt: skip

    def build(seed=None):
        n = max(1, host.get_int("run.cars", 6))
        if args.gym:
            nav = host.get("run.controller", "yours") == "navigator"
            cars = GymCars(args.gym, n, args.vehicle, seed=args.seed if seed is None else seed,
                           navigator=nav)  # fmt: skip
            st.update(cars=cars, speed=[], gap=[], coef={}, world=cars.world)
            if log is not None:
                log.meta.update(world={"source": "gym", "document": str(args.gym)}, cars=n,
                                seed=args.seed if seed is None else seed,
                                controller="navigator" if nav else "yours")  # fmt: skip
            return
        kind = host.get("run.world", "data") or "data"
        try:
            world = make_world(host, kind)
        except Exception as e:  # noqa: BLE001 - say why, and fall back to a world that loads
            host.set("stats.note", f"{kind} world: {e}")
            print(f"cars: the {kind} world failed ({e}); flat ground instead", file=sys.stderr)
            world = flat_world(4.84)
        vehicle = host.get("nav_plant.vehicle", "humvee") or "humvee"
        nav = host.get("run.controller", "yours") == "navigator"
        cars = Cars(world, n, vehicle, seed=args.seed if seed is None else seed, navigator=nav,
                    nav_weights=args.nav_weights, scale=args.nav_scale, app=host.app,
                    prefix=pre)  # fmt: skip
        st.update(cars=cars, speed=[], gap=[], coef={}, world=world)
        show_world(host, world.kind, args.doc)
        if log is not None:
            log.meta.update(
                world=world.info,
                cars=n,
                seed=args.seed if seed is None else seed,
                controller="navigator" if nav else "yours",
                nav_weights=args.nav_weights,
                nav_scale=args.nav_scale,
            )
        if drawn:
            draw_world(host, st)
            draw_cars(host, st)

    def regenerate():
        if (host.get("run.world") or "data") != "generated":
            host.set("run.world", "generated")  # the watch rebuilds
        else:
            build()
        if drawn:
            host.set("world.request", "regenerate")  # the scene's generated_world node redraws it

    host.on("new_goals", lambda: st["cars"].new_goals())
    host.on("reset", lambda: build(int(np.random.default_rng().integers(1 << 30))))
    host.on("regenerate_world", regenerate)
    host.on(
        "load_world",
        lambda: (
            load_data_world(host, host.get("run.uri", AUSTIN), host.get("run.data_path", "")),
            host.set("run.world", "data") if host.get("run.world") != "data" else build(),
        ),
    )

    @host.watch("run.cars")
    @host.watch("run.world")
    @host.watch("run.controller")
    def _rebuild(_v):
        if st["cars"] is not None:
            build()

    try:
        build()
    except Exception as e:  # noqa: BLE001
        print(f"cars: {e}", file=sys.stderr)
        host.close()
        return 1

    @host.on_frame
    def tick(dt):
        cars = st["cars"]
        try:
            cars.pull()
            if not host.get_bool("run.paused"):
                p = {k: host.get_float(f"ctl.{k}", v) for k, v in PARAMS_DEFAULTS(params).items()}
                for _ in range(max(1, host.get_int("run.ticks", 1))):
                    s = cars.state()
                    info = {}
                    if cars.navigator:
                        s = cars.step_navigator()
                        coeffs = s.nav
                    else:
                        r = controller(s, p)
                        info = r[2] if len(r) > 2 and r[2] else {}
                        s = cars.step(r[0], r[1])
                        coeffs = info.get("coeffs", {})
                    st["coef_now"] = coeffs
                    if log is not None:
                        log.record(s, cars.last_command)
                        log.coeffs(coeffs, s.n)
                        if cars.tick % max(1, args.raster_every) == 1 or args.raster_every == 1:
                            log.raster(
                                s.t, **rasters(cars, s, p, controller, cost, host, st, for_log=True)
                            )
                    if host.get_bool("run.regoal", True) and s.reached.any():
                        cars.new_goals(s.reached)
            cars.publish()
            publish(host, cars, st)
            st["frame"] += 1
            if drawn or st["frame"] % 3 == 0:
                show_rasters(host, cars, controller, cost, st, params)
            if drawn:
                if st["frame"] % 15 == 1:
                    show_world(host, cars.world.kind, args.doc, city=False)
                update_scene(host, cars, st)
        except Exception as e:  # noqa: BLE001 - a broken controller ends the run with its reason
            import traceback

            traceback.print_exc()
            st["error"] = str(e)
            host.request_close()

    host.run(fps=30, frames=args.frames or None)
    if args.png and drawn:
        host.snapshot(args.png)
    cars = st["cars"]
    s = cars.state()
    if log is not None:
        path = save_log(
            log, host, args.log,
            ctl={k: host.get(f"ctl.{k}") for k in PARAMS_DEFAULTS(params)},
            run={k: host.get(f"run.{k}") for k in ("cars", "world", "controller", "ticks", "regoal")},
            arrivals=cars.arrivals, collisions=int(cars.collisions.sum()), error=st["error"],
        )  # fmt: skip
        print(f"cars: {len(log)} ticks to {path}")
    print(f"cars: {s.driving} on {s.world}, {s.n} cars, {s.t:.1f} s: {cars.arrivals} arrivals, "
          f"{int(cars.collisions.sum())} car-ticks touching")  # fmt: skip
    code = host.exit_code()
    host.close()
    return exit_code(code, error=st["error"], arrivals=cars.arrivals, min_arrivals=args.min_arrivals)


def PARAMS_DEFAULTS(params):  # noqa: N802 - reads as the constant it returns
    return {k: float(v) for k, v in params.items()}


def load_data_world(host, uri, path=None):
    """Point the document's data.city at ``uri`` (a cvcpkg package, an https archive, a local bundle
    directory), loaded from now on as a city_bundle. ``path`` is the part of an archive to keep (a
    local directory is the bundle itself)."""
    if os.path.isdir(str(uri)):
        path = ""
    elif (
        not path
        and str(uri).startswith("cvcpkg://")
        and str(uri).split("@")[0].endswith("scene-austin-south-small")
    ):
        path = AUSTIN_PATH
    pycvc.data_load(
        host.app, str(uri), path=path or "", decoder="city_bundle", name="city", prefix=host.prefix
    )
    host.set("run.uri", str(uri))


def make_world(host, kind, cell=None) -> World:
    """The world ``kind`` names, from the document: its data.city, its world.* keys, or flat.
    ``cell`` is the grid's cell (m; default: 2 m for a city, the generator's world over 600)."""
    if kind == "data":
        load_data_world(host, host.get("run.uri") or AUSTIN, host.get("run.data_path") or None)
        return world_from_data(
            host.app, f"{host.prefix}.data.city", half=host.get_float("run.crop", 600.0) or None,
            cell=cell or 2.0,
        )
    if kind == "generated":
        params = {}
        for k in WORLD_KEYS:
            v = host.get(f"world.{k}")
            if v in (None, ""):
                continue
            params[k] = v if k in ("terrain", "city", "terrain_file") else float(v)
        return world_from_generated(generate(params, app=host.app), cell=cell)
    return flat_world(4.84)


# ── drawing and panels ─────────────────────────────────────────────────────────────────────────

#: The generated_world node's parts: the node is the terrain, its meshes and water are top-level
#: nodes beside it (cvc/gl/ariadne/world_nodes.h).
GEN_NODES = (
    "gen",
    "gen_props",
    "gen_buildings",
    "gen_roads",
    "gen_foliage",
    "gen_needles",
    "gen_water",
)


def show_world(host, kind, doc, city=True):
    """Show only the world in use. A scene node's `visible:` hides that node alone (cvcGL keeps
    what it drew as children, or as nodes beside it, on screen), so: the generated world's nodes
    are hidden by name, and the document's city is cleared by pointing data.city at a dataset with
    nothing to draw (the document itself, as text), which a `source: { data }` node then empties.
    make_world points it back at run.uri."""
    host.set("view.city", int(kind == "data"))
    host.set("view.generated", int(kind == "generated"))
    if city and kind != "data":
        pycvc.data_load(host.app, str(doc), decoder="text", name="city", prefix=host.prefix)
    if host.mode == "headless":
        return
    for name in GEN_NODES:
        if host.scene.hasGraphics(name):
            host.scene.getGraphics(name).setVisible(kind == "generated")


def draw_world(host, st):
    w = st["world"]
    for name in ("ground", "walls"):
        if host.scene.hasGraphics(name):
            host.scene.removeGraphics(name)
    if w.kind == "flat":
        mnx, mny, mxx, mxy = w.bounds
        host.lab.add_mesh("ground", [mnx, mny, 0, mxx, mny, 0, mxx, mxy, 0, mnx, mxy, 0],
                          [0, 1, 2, 0, 2, 3], color=(0.45, 0.50, 0.40))  # fmt: skip
        host.lab.add_mesh("walls", *_merge([_box(*b[:4], 0.0, b[4]) for b in w.boxes]),
                          color=(0.80, 0.78, 0.74))  # fmt: skip
    st["camera"] = None


_HUMVEE = {"path": None, "dir": None, "models": {}}


def humvee_model_path(host):
    """A local path to the Humvee's glTF model (``Humvee.glb``), or None while there is none:
    an installed ``vehicle-humvee`` package's, else the document's city package's (the Austin
    package carries it). With another world in use, the Austin package's copy is asked for once
    as the document's ``data.vehicles`` (from the data cache after the first download); it is
    read from there when it has loaded. Written once to a temporary directory."""
    if _HUMVEE["path"]:
        return _HUMVEE["path"]
    try:
        from pycvc_gl.scenes import find_vehicle_model

        _HUMVEE["path"] = find_vehicle_model()
    except Exception:  # noqa: BLE001 - none installed: the packages', or boxes
        pass
    if _HUMVEE["path"] or not hasattr(pycvc, "data_file"):
        return _HUMVEE["path"]
    for name in ("city", "vehicles"):
        path = f"{host.prefix}.data.{name}"
        if pycvc.data_status(host.app, path) != "ready":
            continue
        data = pycvc.data_file(host.app, path, "Humvee.glb")
        if data:
            import tempfile

            _HUMVEE["dir"] = _HUMVEE["dir"] or tempfile.TemporaryDirectory(prefix="pycvc-gl-cars-")
            out = os.path.join(_HUMVEE["dir"].name, "Humvee.glb")
            with open(out, "wb") as f:
                f.write(data)
            _HUMVEE["path"] = out
            return out
    if not pycvc.data_status(host.app, f"{host.prefix}.data.vehicles"):
        pycvc.data_load(host.app, AUSTIN, path=AUSTIN_PATH, decoder="files", name="vehicles",
                        prefix=host.prefix)  # fmt: skip
    return None


def humvee_model(host, length):
    """The Humvee's model at ``length`` m (a ``pycvc_gl.vehicle.VehicleModel``), or None."""
    path = humvee_model_path(host)
    if not path:
        return None
    key = round(float(length), 3)
    if key not in _HUMVEE["models"]:
        try:
            from pycvc_gl.vehicle import VehicleModel

            _HUMVEE["models"][key] = VehicleModel(host.app, path, float(length))
        except Exception as e:  # noqa: BLE001 - say why once, then draw boxes
            print(f"cars: the Humvee model did not load ({e}); boxes instead", file=sys.stderr)
            _HUMVEE["models"][key] = None
    return _HUMVEE["models"][key]


def draw_cars(host, st):
    """Each car: the Humvee's model for a Humvee (or nav's bicycle, which has a Humvee's body),
    with a tag in the car's colour above it; a box its size for another preset. Its goal post."""
    cars = st["cars"]
    for name in st["nodes"]:
        if host.scene.hasGraphics(name):
            host.scene.removeGraphics(name)
    st["nodes"], st["tracks"] = [], [[] for _ in range(cars.n)]
    u = float(np.max(cars.length))
    modeled = []
    for i in range(cars.n):
        col = COLORS[i % len(COLORS)]
        preset = cars.preset[i] if i < len(getattr(cars, "preset", [])) else None
        model = humvee_model(host, cars.length[i]) if preset in (None, "humvee") else None
        if model is not None:
            model.add(host.scene, f"car_{i}", col)
            host.lab.add_mesh(f"tag_{i}", *_box(-0.35, -0.35, 0.35, 0.35, 0.0, 0.5), color=col)
            st["nodes"].append(f"tag_{i}")
        else:
            host.lab.add_mesh(
                f"car_{i}", *_car_mesh(cars.length[i], cars.width[i], 0.4 * cars.length[i]),
                color=col,
            )  # fmt: skip
        modeled.append(model is not None)
        host.lab.add_mesh(
            f"goal_{i}", *_box(-0.08 * u, -0.08 * u, 0.08 * u, 0.08 * u, 0, 2 * u), color=col
        )
        st["nodes"] += [f"car_{i}", f"goal_{i}"]
    st["modeled"] = modeled


def update_scene(host, cars, st):
    import pycvc_gl
    from pycvc_gl.vehicle import seat_transform

    s, w = cars.state(), cars.world
    sampler = None if w.height is None else w.z
    if (
        len(st.get("modeled", [])) != s.n
        or len(st.get("sized", [])) != s.n
        or st.get("sized") != list(cars.length)
        or (not all(st["modeled"]) and host.frame_count % 30 == 0 and humvee_model_path(host)
            and any(p in (None, "humvee") for p in cars.preset))  # the model has come since
    ):  # fmt: skip
        draw_cars(host, st)
        st["sized"] = list(cars.length)
    c = s.center
    for i in range(s.n):
        host.scene.getGraphics(f"car_{i}").setTransform(
            seat_transform(c[i, 0], c[i, 1], s.heading[i], sampler)
        )
        if st["modeled"][i]:
            host.scene.getGraphics(f"tag_{i}").setTransform(
                seat_transform(c[i, 0], c[i, 1], s.heading[i], sampler, lift=2.3)
            )
        gx, gy = s.goal[i]
        host.scene.getGraphics(f"goal_{i}").setTransform(
            [1, 0, 0, gx, 0, 1, 0, gy, 0, 0, 1, w.z(gx, gy), 0, 0, 0, 1]
        )
    if host.frame_count % 3 == 0:
        for i in range(s.n):
            st["tracks"][i] = (st["tracks"][i] + [(s.x[i], s.y[i])])[-300:]
    lift = 0.3
    if host.frame_count % 6 == 0:
        for i in range(s.n):
            for name, pts in ((f"track_{i}", st["tracks"][i]),
                              (f"plan_{i}", [] if st["paths"] is None else st["paths"][i])):  # fmt: skip
                if host.scene.hasGraphics(name):
                    host.scene.removeGraphics(name)
                if len(pts) >= 2 and (
                    name.startswith("track") or host.get_bool("show.plans", True)
                ):
                    host.lab.add_path(
                        name, [(x, y, w.z(x, y) + lift) for x, y in pts], COLORS[i % len(COLORS)]
                    )
                    g = host.scene.geometry_node(name)
                    g.setRenderMode(pycvc_gl.GeometryRenderMode_LINES)
                    g.setLineWidth(3.0 if name.startswith("plan") else 1.5)
    mode = host.get("view.camera", "follow")
    k = min(max(0, host.get_int("view.car", 0)), s.n - 1)
    if mode == "follow":
        if st["camera"] != "follow":
            u = float(np.max(cars.length))
            host.set_many({"back": 4.0 * u, "height": 2.0 * u, "look_ahead": 1.5 * u, "look_up": 0.4 * u,
                           "cam_tau": 0.15, "pos_tau": 0.1}, prefix="viewers.main.camera.track")  # fmt: skip
        host.camera.setMode(pycvc_gl.CameraController.Mode_Track)
        host.camera.feedTrackTarget(float(c[k, 0]), float(c[k, 1]), w.z(c[k, 0], c[k, 1]))
    elif st["camera"] != "overview":
        mnx, mny, mxx, mxy = w.bounds
        host.camera.setMode(pycvc_gl.CameraController.Mode_Orbit)
        host.frame_bounds((mnx, mny, 0.0), (mxx, mxy, 40.0))
    st["camera"] = mode


def rasters(cars, s, p, controller, cost, host, st, for_log=False):
    """The chosen car's pictures: its surroundings' signed distance with the controller's predicted
    path, and the controller's cost (or, under the navigator, its belief) there. As arrays for the
    log, and as pictures for the panels."""
    k = chosen(host, s.n)
    size = raster_size(host, np.max(cars.length))
    sim = getattr(cars, "sim", None)  # a GymCars has no navigator of its own to read
    belief = np.asarray(pycvc.nav_sim_world_belief_occ(sim, 0)) if cars.navigator and sim else None
    field = None
    if not cars.navigator and cost is not None:
        def field(X, Y):  # noqa: E306 - the controller's cost around car k
            return cost(s, p, k, X, Y)
    out = raster_arrays(cars.world, s, k, size, belief=belief, cost=field)
    if for_log and not cars.navigator:  # every car's predicted path: (n, k, 2)
        horizon = max(2, int((host.get_float("view.horizon", 3.0) or 3.0) / DT))
        out["plan"] = rollout(controller, s, p, cars.world, steps=horizon, every=2).astype(
            np.float32
        )
    return out


def show_rasters(host, cars, controller, cost, st, params):
    s = cars.state()
    p = {k: host.get_float(f"ctl.{k}", v) for k, v in PARAMS_DEFAULTS(params).items()}
    if not cars.navigator and (host.frame_count % 6 == 0 or st["paths"] is None):
        horizon = max(2, int(host.get_float("view.horizon", 3.0) / DT))
        st["paths"] = rollout(controller, s, p, cars.world, steps=horizon, every=2)
    elif cars.navigator:
        st["paths"] = None
    if host.mode == "headless" or host.frame_count % 4 != 0:
        return
    k = chosen(host, s.n)
    r = rasters(cars, s, p, controller, cost, host, st)
    path = None if st["paths"] is None else st["paths"][k]
    for name, img in raster_pictures(r, s, k, raster_size(host, np.max(cars.length)), path=path).items():
        host.publish_image(name, img)


def plot_coefficients(host, history: dict, coeffs: dict, k: int):
    """Car ``k``'s first four ``coeffs`` (name -> (n,) array) as the cars_coefficients panel's
    plots: each name's last 240 values, kept in ``history`` (a dict the caller holds), at
    ``plot.coef<j>`` with the name at ``plot.coef<j>_name``; unused plots get an empty name."""
    names = list(coeffs)[:4]
    for j in range(4):
        if j < len(names):
            series = history.setdefault(names[j], [])
            series.append(float(np.asarray(coeffs[names[j]], float).reshape(-1)[k]))
            del series[:-240]
            host.set(f"plot.coef{j}_name", names[j])
            host.set_series(f"plot.coef{j}", series)
        else:
            host.set(f"plot.coef{j}_name", "")


def publish(host, cars, st):
    """The readouts: the run's numbers, a row per car, and the plots."""
    s = cars.state()
    k = chosen(host, s.n)
    st["speed"] = (st["speed"] + [float(s.speed[k])])[-240:]
    if s.n > 1:
        d = np.hypot(s.x[:, None] - s.x[None, :], s.y[:, None] - s.y[None, :]) + np.eye(s.n) * 1e9
        st["gap"] = (st["gap"] + [float(d.min())])[-240:]
    host.set_series("plot.speed", st["speed"])
    host.set_series("plot.gap", st["gap"])
    plot_coefficients(host, st["coef"], st.get("coef_now") or {}, k)
    host.set_many({"time": f"{s.t:.1f}", "plant": s.driving, "world": s.world,
                   "arrived": f"{int(s.reached.sum())} / {s.n}", "arrivals": cars.arrivals,
                   "touching": int(cars.collisions.sum()),
                   "controller": "libcvc navigator" if cars.navigator else "yours"}, prefix="stats")  # fmt: skip
    cmd_s, cmd_a = cars.last_command
    host.set_table("carrows", [
        {"car": str(i), "speed": f"{s.speed[i]:.2f}", "steer": f"{math.degrees(s.steer[i]):.0f}",
         "accel": "" if np.isnan(cmd_a[i]) else f"{cmd_a[i]:.2f}", "goal": f"{s.goal_dist[i]:.0f}",
         "status": "touching" if s.collided[i] else ("arrived" if s.reached[i] else "")}
        for i in range(s.n)])  # fmt: skip


def main(argv=None) -> int:
    """``python -m pycvc_gl.cars``: the template's controller in the package's document."""
    from pycvc_gl import cars_template

    return run(cars_template.controller, cars_template.PARAMS, cost=cars_template.cost, argv=argv)


if __name__ == "__main__":
    sys.exit(main())
