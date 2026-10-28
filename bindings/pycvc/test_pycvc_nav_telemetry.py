#!/usr/bin/env python3
"""pycvc sim_world drive telemetry: the tick's books against U_nav from Python.

nav_sim_world_set_capture_drive_telemetry / nav_sim_world_drive_telemetry read
sim_world::drive_telemetry_data, with the books only a world fills: potential_end, projection_work
and context_work (the white paper's context work; libcvc's
NavPotential.SimWorldBooksTheContextWorkOfAGoalChange, through the Python surface). Checks:
  1. no telemetry until capture is on (then the defaults until a captured step), and capturing
     changes no pose;
  2. the dict holds every drive_telemetry field as an (N,) array, the three books included;
  3. no context work before the first captured tick, and no projection work without an override
     or min_gap;
  4. the ledger closes across the tick boundary bit for bit: context_work = potential minus last
     tick's potential_end;
  5. a goal moved between two ticks is context work: the goal spring's energy drops for the two
     retargeted agents, and the others book the same as without the change.
Script-style, like the other pycvc tests; self-skips (exit 0) when the running pycvc predates the
bindings.
"""

import numpy as np
import pycvc

if not hasattr(pycvc, "nav_sim_world_drive_telemetry"):
    print("pycvc lacks nav_sim_world_drive_telemetry — skipping")
    raise SystemExit(0)

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


BOUNDS = (-40.0, -40.0, 40.0, 40.0)  # 81 x 81 one-metre cells
SCALE = 0.05  # normalized = world * 0.05
N = 4
KEYS = {"alpha", "beta", "gamma", "lam_soft", "lam_hard", "mu", "mrisk", "v_risk", "ext_fx",
        "ext_fy", "steer", "curvature", "clearance", "binding", "potential", "u_goal", "u_barrier",
        "u_soft", "u_hard", "u_terrain", "force_x", "force_y", "potential_end", "projection_work",
        "context_work"}


def make_sim():
    """Four agents on open ground (normalized), their goals 2.5 units to the east; the built-in
    policy, nav's kinematic bicycle, the sense frozen."""
    occ = np.zeros((81, 81), np.uint8)
    on = np.array([[-1.2, -0.6], [-1.0, 0.2], [-1.4, 0.6], [-0.8, -0.2]], np.float32)
    goal = on + np.array([2.5, 0.0], np.float32)
    color = np.ones((N, 3), np.float32)
    return pycvc.nav_sim_world_create(
        occ, occ, "", on, goal, color, 81, 81, *BOUNDS, 0.0, 0.0, SCALE, 60.0, 240,
        6.283185307179586, 0.15, 0.35, 0.06, 0.4, 0.165, 0.6, 0.12, 0.12, 0.8, 1, 1, 0.2, 4, 1,
        2.2, -1.4, 8.0, 1, 0.5, 0.15, 4.0)


def run(capture, retarget):
    """Three ticks, then (optionally) agents 0 and 2 retargeted 1 unit ahead of where they are,
    then one more tick: the telemetry of the first, third and fourth ticks, and the end poses."""
    sim = make_sim()
    if capture:
        check(pycvc.nav_sim_world_drive_telemetry(sim) is None, "no telemetry before capture")
        pycvc.nav_sim_world_set_capture_drive_telemetry(sim, 1)
        t0 = pycvc.nav_sim_world_drive_telemetry(sim)
        check(t0 is not None and not np.any(t0["context_work"]) and not np.any(t0["potential"]),
              "before a captured step the telemetry holds its defaults")
    tel = []
    pycvc.nav_sim_world_step(sim)
    tel.append(pycvc.nav_sim_world_drive_telemetry(sim))
    pycvc.nav_sim_world_step(sim)
    pycvc.nav_sim_world_step(sim)
    tel.append(pycvc.nav_sim_world_drive_telemetry(sim))
    if retarget:
        pos = pycvc.nav_sim_world_snapshot(sim)[0]
        for i in (0, 2):
            pycvc.nav_sim_world_retarget(
                sim, i, float(pos[i, 0]) * SCALE + 1.0, float(pos[i, 1]) * SCALE
            )
    pycvc.nav_sim_world_step(sim)
    tel.append(pycvc.nav_sim_world_drive_telemetry(sim))
    return tel, pycvc.nav_sim_world_snapshot(sim)[0]


# 1. capture is opt-in and changes nothing
off_tel, off_pos = run(False, True)
check(all(t is None for t in off_tel), "no telemetry while capture is off")
(first, before, after), on_pos = run(True, True)
check(np.array_equal(off_pos, on_pos), "capturing the books changes no pose")
(_, still_before, still_after), _ = run(True, False)

# 2. the dict
check(set(after) == KEYS, f"telemetry keys {sorted(set(after) ^ KEYS)}")
check(all(v.shape == (N,) for v in after.values()), "every telemetry array is (N,)")
check(after["binding"].dtype == np.uint8, "binding is uint8")
check(all(v.dtype == np.float32 for k, v in after.items() if k != "binding"), "the rest are f32")

# 3. the first tick, and no projection
check(np.all(first["context_work"] == 0), "no context work before the first captured tick")
for t in (first, before, after, still_after):
    check(np.all(t["projection_work"] == 0), "nothing moves an agent after the drive")

# 4. the ledger closes across the tick boundary
for b, a in ((before, after), (still_before, still_after)):
    check(np.array_equal(a["context_work"], a["potential"] - b["potential_end"]),
          "context_work = potential - last tick's potential_end, bit for bit")

# 5. the goal change
for i in range(N):
    cw, cw_still = float(after["context_work"][i]), float(still_after["context_work"][i])
    print(f"  agent {i}{' (retargeted)' if i in (0, 2) else ''}: context work {cw:+.4f} "
          f"(without the goal change {cw_still:+.4f})")
    if i in (0, 2):
        check(cw < 0.0, f"agent {i}: the goal spring's energy drops when its carrot comes in")
        check(abs(cw) > 10.0 * abs(cw_still), f"agent {i}: the goal change dominates")
    else:
        check(cw == cw_still, f"agent {i}: untouched agents book the same context work")

print(f"sim_world drive telemetry bindings: {checks} checks passed")
