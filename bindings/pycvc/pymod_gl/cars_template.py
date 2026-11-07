"""cars.py: N Ackermann cars driven by YOUR controller, in an Ariadne document (cars.ari).

    python cars.py                                    # Austin, 6 Humvees, the controller below
    python cars.py --vehicle kinematic                # nav's bicycle: no physics
    python cars.py --vehicle roboracer --world flat   # the lab's 1/10 car, on a flat square
    python cars.py --world generated --set world.seed=3 --set world.amp=20
    python cars.py --controller navigator             # libcvc's navigator, as a baseline
    python cars.py --headless --frames 1200 --log run.npz   # a batch run: no GL

Made by ``python -m pycvc_gl.cars --copy DIR``: edit ``controller`` below. Every tick (0.05 s) it
gets every car's state as numpy arrays and returns one (steer, accel) per car, which the cars'
plants take in place of the navigator's. The plant is a state-tree setting, ``nav_plant.vehicle``:
the document's Vehicle combo, its ``init:``, ``--vehicle`` or ``--set`` change it, live. So is
every navigator and vehicle setting (``nav.*``, ``nav.vehicle.*``, ``nav_plant.*``) and your own
parameters (``ctl.*``): the Settings and Controller windows bind them.

pycvc_gl.cars (the library) does the rest; its docstrings, and libcvc's docs/ARIADNE.md, "Your
own multi-car controller", say what each piece means.
"""

import sys
from pathlib import Path

import numpy as np

from pycvc_gl.cars import run

# ══ YOUR CONTROLLER ═════════════════════════════════════════════════════════════════════════════

#: Your parameters and their starting values. cars.ari's Controller window has a slider for each
#: (bound to ctl.<name>); the values reach controller() as p[name] every tick.
PARAMS = {
    "speed": 0.4,  # cruise, as a fraction of each car's top speed
    "spacing": 3.0,  # keep this many car lengths from the other cars
    "gain": 1.5,  # steering per radian of heading error
    "look": 4.0,  # react to walls this many car lengths away
}


def controller(s, p):
    """Every car's command for the next tick, from its state ``s`` and your parameters ``p``.

    ``s`` holds (n,) arrays in SI units: ``x``, ``y`` (the rear axle), ``heading`` (rad, 0 along
    +x, counter-clockwise), ``speed`` (m/s along the heading), ``steer`` (the front wheels' angle
    now, rad, + left), ``vx``, ``vy``, ``goal`` (n, 2), ``goal_dist``, ``reached``, ``collided``;
    ``ranges`` (n, R), the distance to anything blocked along each ray at ``ray_angles`` (R,) off
    the heading; the limits ``max_steer``, ``max_accel``, ``max_decel``, ``max_speed``; the sizes
    ``length``, ``width``, ``wheelbase``; and ``t``, ``dt``, ``n``, ``pos`` (n, 2). The other cars
    are the rest of the arrays.

    Return ``(steer, accel)`` as (n,) arrays (rad, + left; m/s^2 along the heading), held to each
    car's limits; a NaN leaves that car to the navigator for the tick. A third value, ``{"coeffs":
    {name: (n,) array}}``, is plotted for the chosen car and logged. This one heads for the goal,
    keeps its spacing, turns away from walls, slows for what is ahead and backs out when wedged.
    """
    L = s.length
    d = s.goal - s.pos
    dist = np.maximum(s.goal_dist, 1e-6)
    pull = d / dist[:, None]

    # Spacing: a push away from each car closer than `spacing` car lengths, stronger nearer.
    rel = s.pos[None, :, :] - s.pos[:, None, :]  # rel[i, j] = car j's position - car i's
    gap = np.hypot(rel[..., 0], rel[..., 1]) + np.eye(s.n) * 1e9
    near = np.clip(1.0 - gap / (p["spacing"] * L[:, None]), 0.0, None)
    push = -(rel / gap[..., None] * near[..., None]).sum(axis=1)

    # Walls: a push away along each ray that sees something blocked within `look` car lengths.
    ang = s.heading[:, None] + s.ray_angles[None, :]
    close = np.clip(1.0 - s.ranges / (p["look"] * L[:, None]), 0.0, None)
    wall = -np.stack([(close * np.cos(ang)).sum(1), (close * np.sin(ang)).sum(1)], axis=1)

    want = pull + 2.0 * push + 1.5 * wall
    err = (np.arctan2(want[:, 1], want[:, 0]) - s.heading + np.pi) % (2 * np.pi) - np.pi
    steer = p["gain"] * err

    ahead = s.ranges[:, np.abs(s.ray_angles) < 0.6].min(axis=1)
    target = p["speed"] * s.max_speed
    target = target * np.clip(dist / (3.0 * L), 0.0, 1.0)  # slow into the goal
    target = target * np.clip(np.cos(err), 0.2, 1.0)  # and through a sharp turn
    target = target * np.clip((ahead - L) / (p["look"] * L), 0.1, 1.0)  # and for a wall ahead
    target = np.where(s.reached, 0.0, target)

    stuck = (ahead < 0.8 * L) & ~s.reached  # wedged against a wall: back out, the other way
    target = np.where(stuck, -0.15 * s.max_speed, target)
    steer = np.where(stuck, -steer, steer)

    accel = 2.0 * (target - s.speed)
    coeffs = {"heading error": err, "target speed": target,
              "spacing push": np.hypot(push[:, 0], push[:, 1]),
              "wall push": np.hypot(wall[:, 0], wall[:, 1])}  # fmt: skip
    return steer, accel, {"coeffs": coeffs}


def cost(s, p, i, X, Y):
    """Optional: the field controller() descends, for car i, at world points (X, Y): drawn in the
    Rasters window and logged. Distance to the goal plus the other cars' spacing; walls are the
    raster's dark cells."""
    L = s.length[i]
    c = np.hypot(X - s.goal[i, 0], Y - s.goal[i, 1]) / L
    for j in range(s.n):
        if j != i:
            g = np.hypot(X - s.x[j], Y - s.y[j]) / (p["spacing"] * L)
            c = c + 4.0 * np.clip(1.0 - g, 0.0, None) ** 2
    return c


if __name__ == "__main__":
    sys.exit(run(controller, PARAMS, cost=cost, doc=str(Path(__file__).with_name("cars.ari"))))
