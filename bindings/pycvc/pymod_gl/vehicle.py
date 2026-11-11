"""Smoothly orient a vehicle to the terrain it drives on.

:class:`VehiclePose` turns a stream of ``(x, y)`` ground positions plus a terrain
height sampler into a smoothed 4x4 model transform (row-major, for
``pycvc_gl`` ``GraphicsNode.setTransform``) that

* **yaws** the vehicle to its heading (estimated from the position stream), and
* **pitches / rolls** it to the terrain **surface normal** — so it banks over a
  crest and leans on a side-slope like a real vehicle —

with frame-rate-independent exponential damping so the pose never snaps (the same
smoothing idea as :class:`pycvc_gl.camera.ChaseCamera`).

The vehicle mesh is assumed modeled with **+X = forward, +Z = up, base at z = 0**.
:class:`VehicleModel` makes a glTF vehicle so, at true scale; :func:`seat_transform` and
:func:`body_transform` place one from a known heading or a physics body's pose.
"""

from __future__ import annotations

import math


def _norm3(v):
    m = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) or 1.0
    return (v[0] / m, v[1] / m, v[2] / m)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _ema(prev, target, tau, dt):
    """Frame-rate-independent exponential smoothing of a scalar toward ``target``."""
    if prev is None:
        return target
    a = 1.0 - math.exp(-dt / max(tau, 1e-6))
    return prev + a * (target - prev)


class VehiclePose:
    """Stateful smoother: feed it ``update(x, y, dt)`` each frame, get back a 4x4
    row-major transform that seats a +X-forward / +Z-up / base-at-0 mesh on the
    terrain, oriented to heading + slope.

    Tunables (all seconds of smoothing time-constant, larger = smoother/laggier):
    ``heading_tau`` (yaw), ``normal_tau`` (pitch/roll). ``normal_step`` is the
    finite-difference distance (world units) used to read the terrain slope;
    ``lift`` raises the body slightly along the normal to avoid z-fighting with the
    road; ``min_speed`` (world units/sec) is the speed below which heading is held.
    """

    def __init__(
        self,
        sampler,
        *,
        lift: float = 0.2,
        heading_tau: float = 0.30,
        normal_tau: float = 0.45,
        normal_step: float = 6.0,
        min_speed: float = 0.5,
    ):
        self._h = sampler
        self._lift = lift
        self._heading_tau = heading_tau
        self._normal_tau = normal_tau
        self._e = normal_step
        self._min_speed = min_speed
        self._prev = None  # previous raw (x, y) for velocity
        self._fwd = None  # smoothed forward XY unit vector
        self._n = None  # smoothed terrain normal (3,)

    def _terrain_normal(self, x, y):
        """Unit surface normal of the height field at ``(x, y)`` (central diff)."""
        e = self._e
        dzdx = (self._h(x + e, y) - self._h(x - e, y)) / (2.0 * e)
        dzdy = (self._h(x, y + e) - self._h(x, y - e)) / (2.0 * e)
        return _norm3((-dzdx, -dzdy, 1.0))

    def update(self, x, y, dt):
        """Advance by ``dt`` seconds to position ``(x, y)``; return a 16-float
        row-major model matrix (list) for ``GraphicsNode.setTransform``."""
        dt = max(dt, 1e-4)

        # -- heading from the (damped) velocity of the position stream --
        if self._prev is None:
            self._prev = (x, y)
        vx = (x - self._prev[0]) / dt
        vy = (y - self._prev[1]) / dt
        self._prev = (x, y)
        speed = math.hypot(vx, vy)
        if speed > self._min_speed:
            hx, hy = vx / speed, vy / speed
            if self._fwd is None:
                self._fwd = (hx, hy)
            else:
                a = 1.0 - math.exp(-dt / max(self._heading_tau, 1e-6))
                fx = self._fwd[0] + a * (hx - self._fwd[0])
                fy = self._fwd[1] + a * (hy - self._fwd[1])
                mag = math.hypot(fx, fy) or 1.0
                self._fwd = (fx / mag, fy / mag)
        elif self._fwd is None:
            self._fwd = (1.0, 0.0)

        # -- damped terrain normal (pitch + roll) --
        tn = self._terrain_normal(x, y)
        if self._n is None:
            self._n = tn
        else:
            self._n = _norm3(
                (
                    _ema(self._n[0], tn[0], self._normal_tau, dt),
                    _ema(self._n[1], tn[1], self._normal_tau, dt),
                    _ema(self._n[2], tn[2], self._normal_tau, dt),
                )
            )
        n = self._n

        # -- orthonormal body frame: X = forward (in the tangent plane), Z = up = n --
        f0 = (self._fwd[0], self._fwd[1], 0.0)
        d = f0[0] * n[0] + f0[1] * n[1] + f0[2] * n[2]
        f = _norm3((f0[0] - d * n[0], f0[1] - d * n[1], f0[2] - d * n[2]))
        left = _norm3(_cross(n, f))  # local +Y = left; (f, left, n) is right-handed

        # -- translation: draped ground point, lifted a touch along the normal --
        z = self._h(x, y)
        tx = x + self._lift * n[0]
        ty = y + self._lift * n[1]
        tz = z + self._lift * n[2]

        # row-major 4x4; matrix columns are the world directions of local X, Y, Z.
        return [
            f[0], left[0], n[0], tx,
            f[1], left[1], n[1], ty,
            f[2], left[2], n[2], tz,
            0.0, 0.0, 0.0, 1.0,
        ]


def seat_transform(x, y, heading, sampler=None, *, lift=0.05, normal_step=2.0):
    """A row-major 4x4 that seats a +X-forward / +Z-up / base-at-0 mesh at ``(x, y)`` facing
    ``heading`` (radians from +X), on the terrain ``sampler(x, y)`` and tilted to its slope
    (flat at z = 0 without one). The unsmoothed counterpart of :class:`VehiclePose`, for a
    simulation that knows each vehicle's heading."""
    if sampler is None:
        n, z = (0.0, 0.0, 1.0), 0.0
    else:
        e = normal_step
        dzdx = (sampler(x + e, y) - sampler(x - e, y)) / (2.0 * e)
        dzdy = (sampler(x, y + e) - sampler(x, y - e)) / (2.0 * e)
        n, z = _norm3((-dzdx, -dzdy, 1.0)), sampler(x, y)
    f0 = (math.cos(heading), math.sin(heading), 0.0)
    d = f0[0] * n[0] + f0[1] * n[1]
    f = _norm3((f0[0] - d * n[0], f0[1] - d * n[1], -d * n[2]))
    left = _norm3(_cross(n, f))
    tx, ty, tz = x + lift * n[0], y + lift * n[1], z + lift * n[2]
    return [
        f[0], left[0], n[0], tx,
        f[1], left[1], n[1], ty,
        f[2], left[2], n[2], tz,
        0.0, 0.0, 0.0, 1.0,
    ]  # fmt: skip


def body_transform(position, rotation, ground_z=None):
    """A row-major 4x4 from a physics body's ``position`` (x, y, z) and ``rotation`` (a unit
    quaternion w, x, y, z), as ``pycvc.physics_world.vehicle_state`` gives them. ``ground_z``
    (optional) puts the mesh's base there instead of at the body's origin (a chassis' origin
    is its centre of mass, above the wheels' contact)."""
    w, qx, qy, qz = (float(c) for c in rotation)
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz, wx, wy, wz = qx * qy, qx * qz, qy * qz, w * qx, w * qy, w * qz
    px, py, pz = (float(c) for c in position)
    if ground_z is not None:
        pz = float(ground_z)
    return [
        1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), px,
        2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), py,
        2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), pz,
        0.0, 0.0, 0.0, 1.0,
    ]  # fmt: skip


class VehicleModel:
    """A vehicle mesh (glTF / GLB, e.g. the ``vehicle-humvee`` package's ``Humvee.glb``) made
    ready to draw at true scale: its smallest extent becomes up (+Z, resting on z = 0), its
    longest forward (+X, scaled to ``length_m``), centred, with the nose picked as the C++ nav
    demos pick it (``load_vehicle_template``: the end whose top sits lower, the hood). Its first
    base-colour texture, if any, comes along.

    ``add(scene, name)`` adds one copy as a scene node (the copies share the mesh); move each
    with ``node.setTransform(seat_transform(...))`` or :class:`VehiclePose`."""

    def __init__(self, app, path, length_m=4.72):
        import numpy as np
        import pycvc

        model = pycvc.load_model(app, str(path))
        g = model.merged()
        if g.num_triangles() == 0:
            raise ValueError(f"VehicleModel: {path} has no triangles")
        pts = np.asarray(g.vertices())
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        ext = hi - lo
        up = int(np.argmin(ext))
        fwd, side = (up + 1) % 3, (up + 2) % 3
        if ext[side] > ext[fwd]:
            fwd, side = side, fwd
        s = float(length_m) / float(ext[fwd]) if ext[fwd] > 1e-9 else 1.0
        cf, cs = 0.5 * (lo[fwd] + hi[fwd]), 0.5 * (lo[side] + hi[side])
        f = pts[:, fwd] - cf
        top = pts[:, up] >= lo[up] + 0.6 * ext[up]  # the upper 40%
        front = 1.0 if (f > 0).sum() < (f <= 0).sum() else -1.0  # the lighter half
        if (top & (f > 0)).sum() > 8 and (top & (f <= 0)).sum() > 8:
            front = 1.0 if f[top & (f > 0)].mean() < abs(f[top & (f <= 0)].mean()) else -1.0
        out = np.empty_like(pts)
        out[:, 0] = f * s * front
        out[:, 1] = (pts[:, side] - cs) * s
        out[:, 2] = (pts[:, up] - lo[up]) * s
        pts[:] = out  # vertices() is the geometry's own buffer
        normals = np.asarray(g.get_normals(), float).reshape(-1, 3)
        if len(normals) == len(pts):  # the same axis change, without the scale
            nn = np.stack([normals[:, fwd] * front, normals[:, side], normals[:, up]], axis=1)
            g.set_normals(nn.ravel().tolist())
        self.geometry = g
        self.length_m = float(length_m)
        self.size = tuple(float(v) for v in ext[[fwd, side, up]] * s)  # length, width, height
        self.texture = None
        for mat in model.material_list():
            if mat.has_base_color_texture():
                self.texture = mat.base_color_texture_image()
                break

    def add(self, scene, name, color=(0.42, 0.45, 0.33)):
        """Add a copy as the scene node ``name`` (textured when the model has a texture, else a
        flat ``color``); returns the node."""
        node = scene.addGraphics(name, self.geometry)
        gn = scene.geometry_node(name)
        if self.texture is not None:
            gn.setUseSingleColor(False)
            gn.set_texture(self.texture)
        else:
            gn.setUseSingleColor(True)
            gn.setColor(*color)
        gn.setAmbient(0.32)
        gn.setDiffuse(0.9)
        gn.setSpecular(0.2)
        return node
