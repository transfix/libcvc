#!/usr/bin/env python3
"""pycvc.nav_sim_world_override: a caller's (steer, accel) per agent in place of the navigator's.

Checks:
  1. on nav's kinematic bicycle an overridden agent integrates the bicycle with the command, to
     float precision against the same bicycle in numpy (a straight run, then a turn);
  2. a NaN leaves that agent to the navigator, and the call returns how many agents took a command;
  3. a command holds for one step: an agent held at rest by (0, 0) stays put, and drives off on
     the navigator's command once the overrides stop;
  4. the steer is held to the lock and the speed to the top speed;
  5. arrays of the wrong length or rank are errors;
  6. nav_sim_world_set_pose places an agent on nav's bicycle (the snapshot reads it back), and an
     index out of range is an error;
  7. with Jolt, a plant set after set_pose attaches the Humvee at that pose, set_pose then does
     nothing (False), and an overridden Humvee drives off straight along its heading, then turns
     left on a positive steer.
Script-style, like the other pycvc tests; self-skips (exit 0) when the running pycvc predates the
bindings.
"""

import math

import numpy as np
import pycvc

if not hasattr(pycvc, "nav_sim_world_override") or not hasattr(pycvc, "nav_sim_world_set_pose"):
    print("pycvc lacks nav_sim_world_override / nav_sim_world_set_pose — skipping")
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
SCALE = 0.05  # normalized units per metre
DT, NSUB, VMAX, L, LOCK = 0.06, 6, 0.4, 0.165, 0.6  # the world's vehicle (normalized)
OPEN = np.zeros((81, 81), np.uint8)


def make_sim(starts, goals):
    """test_pycvc_nav_routes.make_sim's world with agents from `starts` to `goals` (normalized):
    the built-in policy, nav's kinematic bicycle, reverse allowed (speed down to -0.25 vmax)."""
    n = len(starts)
    return pycvc.nav_sim_world_create(
        OPEN, OPEN, "", np.array(starts, np.float32), np.array(goals, np.float32),
        np.ones((n, 3), np.float32), 81, 81, *BOUNDS, 0.0, 0.0, SCALE, 60.0, 240,
        6.283185307179586, 0.15, 0.35, DT, VMAX, L, LOCK, 0.12, 0.12, 0.8, NSUB, 1, 0.2, 4, 1,
        2.2, -1.4, 8.0, 1, 0.5, 0.15, 4.0)


def bicycle(x, y, th, sp, steer, accel, ticks):
    """libcvc's kinematic_step (plant_types.h) in float32, `ticks` times: normalized units."""
    f = np.float32
    x, y, th, sp = f(x), f(y), f(th), f(sp)
    h, t = f(DT / NSUB), f(math.tan(steer))
    for _ in range(ticks):
        for _ in range(NSUB):
            sp = f(min(max(sp + h * f(accel), f(-0.25 * VMAX)), f(VMAX)))
            th = f(th + h * (sp / f(L)) * t)
            x = f(x + h * sp * f(math.cos(th)))
            y = f(y + h * sp * f(math.sin(th)))
    return x, y, th, sp


def norm_state(sim, i):
    """Agent i's (x, y, heading, speed), normalized, from the snapshot (world metres, m/s)."""
    pos, head, speed = pycvc.nav_sim_world_snapshot(sim)[:3]
    return pos[i, 0] * SCALE, pos[i, 1] * SCALE, float(head[i]), speed[i] * SCALE


# 1-2. a straight run and a turn on the bicycle; agent 1 left to the navigator
sim = make_sim([[-1.0, 0.5], [-1.0, -0.5]], [[1.0, 0.5], [1.0, -0.5]])
x, y, th, sp = norm_state(sim, 0)
nan = float("nan")
for steer, accel, ticks in ((0.0, 0.05, 10), (0.3, 0.0, 10)):
    for _ in range(ticks):
        n = pycvc.nav_sim_world_override(sim, [steer, nan], np.array([accel, 0.0]))
        pycvc.nav_sim_world_step(sim)
    check(n == 1, f"one agent took the command, not {n}")
    x, y, th, sp = bicycle(x, y, th, sp, steer, accel, ticks)
    gx, gy, gth, gsp = norm_state(sim, 0)
    err = max(abs(gx - x), abs(gy - y), abs(gth - th), abs(gsp - sp))
    check(err < 1e-5, f"steer {steer}, accel {accel}: the bicycle's state, off by {err:.2e}")
check(sp > 0.02 and th > 0.02, f"it sped up and turned left (speed {sp:.3f}, heading {th:.3f})")
moved = np.hypot(*(pycvc.nav_sim_world_snapshot(sim)[0][1] - np.array([-20.0, -10.0])))
check(moved > 0.5, f"the navigator drove agent 1 ({moved:.2f} m)")

# 3. one step only
sim = make_sim([[-1.0, 0.0]], [[1.0, 0.0]])
start = pycvc.nav_sim_world_snapshot(sim)[0][0].copy()
for _ in range(5):
    pycvc.nav_sim_world_override(sim, [0.0], [0.0])
    pycvc.nav_sim_world_step(sim)
pos, _, speed = pycvc.nav_sim_world_snapshot(sim)[:3]
check(np.allclose(pos[0], start) and speed[0] == 0.0, "held at rest by (0, 0)")
for _ in range(10):
    pycvc.nav_sim_world_step(sim)
pos = pycvc.nav_sim_world_snapshot(sim)[0]
check(pos[0, 0] - start[0] > 0.1, f"then the navigator drives it ({pos[0, 0] - start[0]:.2f} m)")

# 4. the lock and the top speed
a = make_sim([[-1.0, 0.0]], [[1.0, 0.0]])
b = make_sim([[-1.0, 0.0]], [[1.0, 0.0]])
for _ in range(20):
    pycvc.nav_sim_world_override(a, [5.0], [0.05])
    pycvc.nav_sim_world_override(b, [LOCK], [0.05])
    pycvc.nav_sim_world_step(a)
    pycvc.nav_sim_world_step(b)
check(np.allclose(norm_state(a, 0), norm_state(b, 0), atol=1e-6),
      "a steer past the lock is the lock")
for _ in range(100):
    pycvc.nav_sim_world_override(a, [0.0], [10.0])
    pycvc.nav_sim_world_step(a)
top = norm_state(a, 0)[3]
check(abs(top - VMAX) < 1e-5, f"held to the top speed ({top:.4f} of {VMAX})")

# 5. malformed arrays
expect_raises(lambda: pycvc.nav_sim_world_override(a, [0.0, 0.0], [0.0, 0.0]), "two for one agent")
expect_raises(lambda: pycvc.nav_sim_world_override(a, [[0.0]], [0.0]), "a 2-D steer")
expect_raises(lambda: pycvc.nav_sim_world_override(a, [0.0], "x"), "a string accel")


# 6. set_pose on nav's bicycle
sim = make_sim([[-1.0, 0.0], [1.0, 0.0]], [[1.0, 0.0], [-1.0, 0.0]])
check(pycvc.nav_sim_world_set_pose(sim, 1, 0.5, -0.25, 1.0), "placed on the bicycle")
gx, gy, gth, gsp = norm_state(sim, 1)
check(abs(gx - 0.5) < 1e-6 and abs(gy + 0.25) < 1e-6 and abs(gth - 1.0) < 1e-6 and gsp == 0.0,
      f"the snapshot reads the pose back ({gx:.3f}, {gy:.3f}, {gth:.3f}, {gsp:.3f})")
expect_raises(lambda: pycvc.nav_sim_world_set_pose(sim, 2, 0.0, 0.0, 0.0), "agent 2 of 2")


# 7. a Jolt Humvee on a plant
def has_jolt():
    return (hasattr(pycvc, "nav_plant") and "jolt" in pycvc.physics_backends()
            and pycvc.physics_backend_info("jolt")["available"])


if has_jolt():
    sim = make_sim([[-1.0, 0.0]], [[1.0, 0.0]])
    pycvc.nav_sim_world_set_pose(sim, 0, -1.0, 0.0, 0.5)  # facing up and to the right
    w = pycvc.physics_world()
    ground = w.add_material("ground", friction=0.9)
    w.add_box((60.0, 60.0, 0.5), position=(0.0, 0.0, -0.5), motion="static", material=ground)
    plant = pycvc.nav_plant(w, [pycvc.vehicle_desc.humvee()])
    pycvc.nav_sim_world_set_plant(sim, plant)
    x0, y0, th0, _ = norm_state(sim, 0)
    check(abs(th0 - 0.5) < 0.02, f"the plant attached the Humvee at its heading ({th0:.3f})")
    check(not pycvc.nav_sim_world_set_pose(sim, 0, 0.0, 0.0, 0.0), "the plant owns its pose")
    for _ in range(40):  # 2.4 s at 1.5 m/s^2
        pycvc.nav_sim_world_override(sim, [0.0], [1.5 * SCALE])
        pycvc.nav_sim_world_step(sim)
    x1, y1, th1, sp1 = norm_state(sim, 0)
    along = ((x1 - x0) * math.cos(0.5) + (y1 - y0) * math.sin(0.5)) / SCALE
    across = (-(x1 - x0) * math.sin(0.5) + (y1 - y0) * math.cos(0.5)) / SCALE
    check(along > 1.0 and abs(across) < 0.3,
          f"the Humvee drives off straight ({along:.2f} m along, {across:.2f} m across)")
    check(sp1 / SCALE > 1.0, f"at speed ({sp1 / SCALE:.2f} m/s)")
    for _ in range(30):
        pycvc.nav_sim_world_override(sim, [0.3], [0.0])
        pycvc.nav_sim_world_step(sim)
    th2 = norm_state(sim, 0)[2]
    check(th2 - th1 > 0.1, f"a positive steer turns it left ({th2 - th1:.3f} rad)")
else:
    print("Jolt is not available — skipping check 7")

print(f"pycvc nav_sim_world_override / set_pose: {checks} checks passed")
