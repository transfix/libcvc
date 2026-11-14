#!/usr/bin/env python3
"""pycvc.plant_host (the plant chosen by name, or live from the state tree) and pycvc.generate_world
(libcvc's terrain generator, the generated_world node's world).

Checks:
  1. generate_world is a pure function of its params: the same params make the same terrain and
     items; the node's key names (half, amp, water_level) and the fields' (half_m, ...) are the
     same param; an unknown param or a wrong type is an error; info() reports what was made;
  2. plant_host: the presets, check() of a good and a bad name, use() on kinematic (no plant: state,
     limits and vehicle_state are None);
  3. with Jolt: use("humvee") puts the agents on Jolt Humvees (kind, limits in nav's frame,
     vehicle_state), and on a generated world (set_geometry) each one starts on the terrain under
     it and drives on an override;
  4. pull() from the state tree: nav_plant.vehicle switches the plant live (the agent keeps its
     pose), an unknown name is ignored, and .driving says what it drives;
  5. lifetimes: the host holds the sim_world it drives, and a host dropped first puts it back on
     the bicycle (the sim keeps stepping).
Script-style, like the other pycvc tests; self-skips (exit 0) when the running pycvc predates the
bindings.
"""

import gc
import math

import numpy as np
import pycvc

if not hasattr(pycvc, "plant_host") or not hasattr(pycvc, "generate_world"):
    print("pycvc lacks plant_host / generate_world — skipping")
    raise SystemExit(0)

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


def expect_raises(fn, what):
    try:
        fn()
    except (RuntimeError, ValueError, TypeError):
        return
    raise AssertionError(what + ": no error raised")


S = 0.05  # normalized units per metre


def make_sim(occ, bounds, starts, goals):
    rows, cols = occ.shape
    on = (np.asarray(starts, np.float32)) * S
    goal = (np.asarray(goals, np.float32)) * S
    n = len(starts)
    return pycvc.nav_sim_world_create(
        occ,
        occ,
        "",
        on,
        goal,
        np.ones((n, 3), np.float32),
        rows,
        cols,
        *bounds,
        0.0,
        0.0,
        S,
        60.0,
        120,
        2 * math.pi,
        1.2 * S,
        3.0 * S,
        0.05,
        24.6 * S,
        3.3 * S,
        0.457,
        2.0 * S,
        3.0 * S,
        0.8,
        5,
        1,
        6.0 * S,
        4,
        1,
        2.2,
        -1.4,
        8.0,
        1,
        0.5,
        0.15,
        4.0,
    )


# 1. generate_world
P = {"seed": 11, "half": 200, "buildings": 30, "trees": 40, "rocks": 10, "amp": 20}
a = pycvc.generate_world(P)
b = pycvc.generate_world(
    {**{k: v for k, v in P.items() if k not in ("half", "amp")}, "half_m": 200, "amp_m": 20}
)
pts = [(-120.0, 35.0), (0.0, 0.0), (77.0, -140.0)]
check(all(a.height(x, y) == b.height(x, y) for x, y in pts), "same params, same terrain")
check(list(a.world().names()) == list(b.world().names()), "same params, same items")
c = pycvc.generate_world({**P, "seed": 12})
check(any(a.height(x, y) != c.height(x, y) for x, y in pts), "another seed, another terrain")
info = a.info()
check(info["seed"] == 11 and info["half_m"] == 200.0 and info["terrain"] == "fbm", f"info {info}")
check(a.terrain().bounds2d() == (-200.0, -200.0, 200.0, 200.0), f"bounds {a.terrain().bounds2d()}")
check(a.world().has_terrain(), "its world has the terrain to collide and navigate")
expect_raises(lambda: pycvc.generate_world({"sead": 1}), "an unknown param")
expect_raises(lambda: pycvc.generate_world({"half": "big"}), "a number param as a string")
expect_raises(lambda: pycvc.generate_world({"terrain": "nope"}), "an unknown generator")
check(info["edge"] == "wall", "walls at the edge by default")
s = pycvc.generate_world({**P, "edge": "shore"})
check(s.info()["edge"] == "shore" and s.height(199.9, 0.0) < -5.0, "a shore: the sea at the edge")
check(s.height(0.0, 0.0) == a.height(0.0, 0.0), "a shore: the same ground inland")
expect_raises(lambda: pycvc.generate_world({"edge": "moat"}), "an unknown edge")

# 2. plant_host on the bicycle
check(set(pycvc.plant_host.presets()) >= {"humvee", "roboracer"}, "the presets")
pycvc.plant_host.check("humvee,roboracer")
expect_raises(lambda: pycvc.plant_host.check("tank"), "an unknown vehicle")
FLAT = np.zeros((81, 81), np.uint8)
FB = (-40.0, -40.0, 40.0, 40.0)
sim = make_sim(FLAT, FB, [(-20.0, 0.0)], [(20.0, 0.0)])
host = pycvc.plant_host()
check(host.use(sim, "kinematic") == "kinematic" and host.kind() == "kinematic", "the bicycle")
check(
    host.state(0) is None and host.limits(0) is None and host.vehicle_state(0) is None,
    "no plant to read on the bicycle",
)

if pycvc.plant_host.available():
    # 3. Jolt Humvees, then on a generated world
    check(host.use(sim, "humvee") == "humvee" and host.kind() == "jolt", "Jolt Humvees")
    lim = host.limits(0)
    check(
        abs(lim["wheelbase"] / S - 3.30) < 1e-3 and abs(lim["vmax"] / S - 24.6) < 1e-3,
        f"the Humvee's limits, normalized ({lim['wheelbase'] / S:.3f} m, {lim['vmax'] / S:.1f} m/s)",
    )
    check(host.vehicle_state(0) is not None and host.state(0) is not None, "its state")
    expect_raises(lambda h=host: h.state(5), "agent 5 of 1")

    occ = np.ascontiguousarray(
        pycvc.nav_world_rasterize(a.world(), 81, 81, -200.0, -200.0, 200.0, 200.0)["occupancy"],
        np.uint8,
    )
    free = np.argwhere(occ == 0)
    r, cc = free[len(free) // 2]
    x0, y0 = -200.0 + cc * 5.0, -200.0 + r * 5.0
    gsim = make_sim(occ, (-200.0, -200.0, 200.0, 200.0), [(x0, y0)], [(x0 + 50.0, y0)])
    ghost = pycvc.plant_host()
    ghost.set_geometry(a.world())
    check(ghost.use(gsim, "humvee") == "humvee", "Humvees on the generated world")
    z = ghost.vehicle_state(0)["position"][2]
    ground = a.height(x0 + 1.65, y0)  # the chassis' origin: half a wheelbase ahead of the axle
    check(abs(z - ground) < 2.5, f"it starts on the terrain (z {z:.2f}, ground {ground:.2f})")
    for _ in range(30):
        pycvc.nav_sim_world_override(gsim, [0.0], [1.5 * S])
        pycvc.nav_sim_world_step(gsim)
    sp = pycvc.nav_sim_world_snapshot(gsim)[2][0]
    check(sp > 0.5, f"and drives on an override ({sp:.2f} m/s)")

    # 4. live, from the state tree
    app = pycvc.make_app()
    pycvc.state_set(app, "cars.nav_plant.vehicle", "roboracer")
    pos0 = pycvc.nav_sim_world_snapshot(sim)[0][0].copy()
    host.pull(app, "cars", sim)
    check(host.vehicle() == "roboracer" and host.driving() == "roboracer", "switched to RoboRacers")
    check(pycvc.state_get(app, "cars.nav_plant.driving") == "roboracer", ".driving says so")
    check(np.allclose(pycvc.nav_sim_world_snapshot(sim)[0][0], pos0, atol=0.2), "same pose")
    check(abs(host.limits(0)["wheelbase"] / S - 0.324) < 1e-3, "the RoboRacer's wheelbase")
    pycvc.state_set(app, "cars.nav_plant.vehicle", "tank")
    host.pull(app, "cars", sim)
    check(host.driving() == "roboracer", "an unknown name is ignored")
    pycvc.state_set(app, "cars.nav_plant.vehicle", "kinematic")
    host.pull(app, "cars", sim)
    check(host.kind() == "kinematic", "back to the bicycle, live")
    pycvc.state_set(app, "cars.nav_plant.vehicle", "dynamic:roboracer")
    host.pull(app, "cars", sim)
    check(host.kind() == "dynamic" and host.body(0) is not None, "the dynamic single track, live")
else:
    print("Jolt is not available — skipping checks 3 and 4")

# 5. lifetimes: the host drops first; the sim keeps stepping on the bicycle
host.use(sim, "humvee" if pycvc.plant_host.available() else "kinematic")
del host
gc.collect()
for _ in range(5):
    pycvc.nav_sim_world_step(sim)
check(pycvc.nav_sim_world_plant_kind(sim) == "kinematic", "the dropped host left it on the bicycle")
held = pycvc.plant_host()
held.use(sim, "kinematic")
del sim
gc.collect()
check(held.kind() == "kinematic", "the host holds the sim it drives")
del held

print(f"pycvc plant_host / generate_world: {checks} checks passed")
