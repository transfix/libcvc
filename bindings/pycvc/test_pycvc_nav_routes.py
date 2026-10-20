#!/usr/bin/env python3
"""pycvc vehicle routes: plan_vehicle_route and route_follower from Python (the plant's P9 R1/R2).

Checks:
  1. a route on open ground goes straight to the goal, forward, with no cusp;
  2. in a corridor too narrow to turn around in, a goal behind the vehicle is reached backing up,
     and every pose keeps clear of the walls;
  3. the option dicts are validated (an unknown key or a non-number is an error);
  4. route_clearance is positive in the open and negative inside a wall;
  5. a route_follower drives a sim_world agent along its route to the goal, and its status reads
     the route's legs;
  6. the follower keeps its sim_world alive after the Python name for the world is gone;
  7. an agent index out of range and a malformed pose array are errors;
  8. nav_sim_world_belief_occ copies a belief plane's planning raster (what a route is planned on);
  9. nav_sim_world_route_vehicle derives the route vehicle from the plant (none on nav's bicycle),
     and with Jolt a Humvee drives its route to the end on the plant.
Script-style, like the other pycvc tests; self-skips (exit 0) when the running pycvc predates the
bindings.
"""

import gc
import math

import numpy as np
import pycvc

if not hasattr(pycvc, "nav_plan_vehicle_route"):
    print("pycvc lacks nav_plan_vehicle_route — skipping")
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
    except RuntimeError:
        return
    raise AssertionError(what + ": no error raised")


BOUNDS = (-40.0, -40.0, 40.0, 40.0)  # 81 x 81 one-metre cells
CAR = {"min_radius": 4.8, "disc_offsets": [0.0, 2.0], "disc_radius": 0.6}


def plan(occ, start, goal, vehicle=CAR, options=None):
    return pycvc.nav_plan_vehicle_route(occ, *BOUNDS, *start, *goal, vehicle, options)


# 1. open ground
open_ground = np.zeros((81, 81), np.uint8)
r = plan(open_ground, (-20.0, 0.0, 0.0), (20.0, 0.0))
p = r["poses"]
check(r["reached"], "a route on open ground")
check(p.shape[1] == 4 and p.dtype == np.float64, "poses are (M,4) float64")
check(np.allclose(p[0, :3], [-20.0, 0.0, 0.0]), "the route starts at the start pose")
check(math.hypot(p[-1, 0] - 20.0, p[-1, 1]) <= 2.0, "it ends within the goal tolerance")
check(r["cusps"] == 0 and r["reverse_length"] == 0.0 and not p[:, 3].any(), "all forward")
check(abs(r["length"] - 40.0) < 4.0, f"about 40 m long ({r['length']:.1f})")

# 2. a corridor 7 m wide: a forward U-turn needs 2 x 4.8 m, so heading back means backing up
corridor = np.ones((81, 81), np.uint8)
corridor[37:44, 5:76] = 0  # rows y in [-3, 3], x in [-35, 35]
r = plan(corridor, (0.0, 0.0, 0.0), (-25.0, 0.0))
p = r["poses"]
check(r["reached"], "a route that turns around in the corridor")
# A point goal behind it: backing straight up the corridor is the route (no cusp needed).
check(r["reverse_length"] > 0.0 and p[1:, 3].all(), "it backs up the corridor")
clear = [pycvc.nav_route_clearance(corridor, *BOUNDS, x, y, h, CAR) for x, y, h, _ in p]
check(min(clear) >= 0.0, f"every pose keeps clear of the walls (min {min(clear):.2f})")

# 3. the option dicts
expect_raises(lambda: plan(open_ground, (0.0, 0.0, 0.0), (5.0, 0.0), {"min_radus": 4.8}),
              "an unknown vehicle key")
expect_raises(lambda: plan(open_ground, (0.0, 0.0, 0.0), (5.0, 0.0), CAR, {"step": "x"}),
              "a non-number option")
expect_raises(lambda: pycvc.nav_route_follower_create(None), "a follower without a world")

# 4. clearance
check(pycvc.nav_route_clearance(corridor, *BOUNDS, 0.0, 0.0, 0.0, CAR) > 0.0, "clear in the open")
check(pycvc.nav_route_clearance(corridor, *BOUNDS, 0.0, 20.0, 0.0, CAR) < 0.0, "inside a wall")


# 5. a follower drives a sim_world agent home
def make_sim(occ):
    """test_pycvc_physics.make_sim's world on `occ`: one agent from (-20, 0) to (20, 0) m (scale
    0.05: normalized -1 -> 1), the built-in policy, nav's kinematic bicycle."""
    on = np.array([[-1.0, 0.0]], np.float32)
    goal = np.array([[1.0, 0.0]], np.float32)
    color = np.ones((1, 3), np.float32)
    return pycvc.nav_sim_world_create(
        occ, occ, "", on, goal, color, 81, 81, *BOUNDS, 0.0, 0.0, 0.05, 60.0, 240,
        6.283185307179586, 0.15, 0.35, 0.06, 0.4, 0.165, 0.6, 0.12, 0.12, 0.8, 6, 1, 0.2, 4, 1,
        2.2, -1.4, 8.0, 1, 0.5, 0.15, 4.0)


sim = make_sim(open_ground)
b = pycvc.nav_sim_world_belief_occ(sim, 0)
check(b.shape == (81, 81) and b.dtype == np.uint8 and not b.any(), "plane 0's raster, all free")
expect_raises(lambda: pycvc.nav_sim_world_belief_occ(sim, 1), "plane 1 of 1")
follow = pycvc.nav_route_follower_create(sim, {"manoeuvre_speed": 1.5})
st = pycvc.nav_route_follower_status(follow)
check(st["following"].shape == (1,) and not st["following"][0], "no route yet")
route = plan(open_ground, (-20.0, 0.0, 0.0), (20.0, 0.0))
pycvc.nav_route_follower_set(follow, 0, route["poses"])
st = pycvc.nav_route_follower_status(follow)
check(st["following"][0] and st["legs"][0] == 1 and st["leg"][0] == 0, "one forward leg")
check(not st["reversing"][0] and not st["done"][0], "driving it forward, not done")

# 6. the follower holds the world: with the Python name for it gone, apply still reads it
del sim
gc.collect()
pycvc.nav_route_follower_apply(follow)
check(pycvc.nav_route_follower_status(follow)["following"][0], "the world outlives its name")
del follow
gc.collect()

# 5 (cont.). drive to the goal with a world we keep a name for
sim = make_sim(open_ground)
follow = pycvc.nav_route_follower_create(sim)
pycvc.nav_route_follower_set(follow, 0, route["poses"])
# The follower retargets the agent to a carrot along the route, so the snapshot's `reached` is
# "at the carrot"; the route is finished when the follower says so.
for tick in range(2500):
    pycvc.nav_route_follower_apply(follow)
    pycvc.nav_sim_world_step(sim)
    if pycvc.nav_route_follower_status(follow)["done"][0]:
        break
pos = pycvc.nav_sim_world_snapshot(sim)[0]
check(bool(pycvc.nav_route_follower_status(follow)["done"][0]),
      f"the agent finishes its route (tick {tick})")
# Done within the sim's reach_tol (0.2 normalized = 4 m) of the route's end, which is within the
# route's goal tolerance (2 cells = 2 m) of the goal.
check(math.hypot(float(pos[0, 0]) - 20.0, float(pos[0, 1])) < 6.0,
      f"near (20, 0) m: {pos[0].tolist()}")

# 7. errors
expect_raises(lambda: pycvc.nav_route_follower_set(follow, 1, route["poses"]), "agent 1 of 1")
expect_raises(lambda: pycvc.nav_route_follower_set(follow, 0, np.zeros((5, 2))), "(M,2) poses")
expect_raises(lambda: pycvc.nav_route_follower_clear(follow, -1), "agent -1")
expect_raises(lambda: pycvc.nav_route_follower_apply(sim), "a world handle is not a follower")
pycvc.nav_route_follower_clear(follow, 0)
check(not pycvc.nav_route_follower_status(follow)["following"][0], "cleared")

# 9. the route vehicle from the plant: none on nav's bicycle; on a Jolt plant, the vehicle's
#    turning radius and body, and a route it drives to the end
check(pycvc.nav_sim_world_route_vehicle(sim, 0) is None, "no plant, no route vehicle")
expect_raises(lambda: pycvc.nav_sim_world_route_vehicle(sim, 3), "agent 3 of 1")
if "jolt" in pycvc.physics_backends() and pycvc.physics_backend_info("jolt")["available"]:
    w = pycvc.physics_world()
    gm = w.add_material("ground", friction=0.9, restitution=0.0, rolling_resistance=0.015)
    w.add_box((150.0, 150.0, 0.5), position=(0.0, 0.0, -0.5), motion="static", material=gm)
    sim = make_sim(open_ground)
    plant = pycvc.nav_plant(w, [pycvc.vehicle_desc.humvee()])
    pycvc.nav_sim_world_set_plant(sim, plant)
    rv = pycvc.nav_sim_world_route_vehicle(sim, 0)
    check(rv is not None and rv["min_radius"] > 0.0 and rv["wheelbase"] > 0.0, "a turning radius")
    check(abs(rv["min_radius"] - rv["wheelbase"] / math.tan(rv["max_steer"])) < 1e-9,
          "min_radius = wheelbase / tan(max_steer)")
    check(rv["reach"] > 0.0, "the sim's reach in world units")
    car = {k: rv[k] for k in ("min_radius", "disc_offsets", "disc_radius")}
    pos0, head0 = pycvc.nav_sim_world_snapshot(sim)[:2]
    r = pycvc.nav_plan_vehicle_route(open_ground, *BOUNDS, float(pos0[0, 0]), float(pos0[0, 1]),
                                     float(head0[0]), 20.0, 0.0, car)
    check(r["reached"], "a route for the Humvee")
    follow = pycvc.nav_route_follower_create(
        sim, {"wheelbase": rv["wheelbase"], "max_steer": rv["max_steer"]})
    pycvc.nav_route_follower_set(follow, 0, r["poses"])
    for tick in range(3000):
        pycvc.nav_route_follower_apply(follow)
        pycvc.nav_sim_world_step(sim)
        if pycvc.nav_route_follower_status(follow)["done"][0]:
            break
    pos = pycvc.nav_sim_world_snapshot(sim)[0]
    check(bool(pycvc.nav_route_follower_status(follow)["done"][0]),
          f"the Humvee finishes its route on the plant (tick {tick})")
    check(math.hypot(float(pos[0, 0]) - 20.0, float(pos[0, 1])) < rv["reach"] + 2.0,
          f"the Humvee ends near (20, 0) m: {pos[0].tolist()}")
else:
    print("no Jolt in this build: the plant checks are skipped")

print(f"vehicle route bindings: {checks} checks passed")
