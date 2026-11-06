#!/usr/bin/env python3
"""pycvc: the sim_world's risk governor from Python (``nav_sim_world_set_risk_governor``).

Water (hard material, risk 1) square across an agent's path. The material's forces push straight
back along its heading, so they do not steer it, and take less off its acceleration than its goal
pull has to spare: it crosses at its top speed. With the governor on it crosses at the speed the
water's risk allows. Script-style, like the other pycvc tests; self-skips (exit 0) when the running
pycvc predates the binding. On Windows, CVC_DLL_DIRS (os.pathsep-separated) names the directories
pycvc's DLLs load from.
"""

import os
import sys

if sys.platform == "win32":
    for _d in os.environ.get("CVC_DLL_DIRS", "").split(os.pathsep):
        if _d and os.path.isdir(_d):
            os.add_dll_directory(_d)

import numpy as np

import pycvc

if not hasattr(pycvc, "nav_sim_world_set_risk_governor"):
    print("pycvc lacks nav_sim_world_set_risk_governor — skipping")
    raise SystemExit(0)

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


def raises(fn, what):
    try:
        fn()
    except (RuntimeError, ValueError):
        global checks
        checks += 1
        return
    raise AssertionError(what + ": no error raised")


# 81 x 81 one-metre cells, scale 0.05 (test_pycvc_physics.py's world): the agent 36 m west of
# x = 0, its goal 36 m east, and water 9 m wide across the map between them.
N = 81
open_field = np.zeros((N, N), np.uint8)
risk = np.zeros((N, N), np.float32)
hard = np.zeros((N, N), np.uint8)
hard[:, 36:45] = 1
risk[hard != 0] = 1.0
VMAX = 0.4  # normalized: 8 m/s


def make_sim():
    sim = pycvc.nav_sim_world_create(
        open_field, open_field, "", np.array([[-1.8, 0.0]], np.float32),
        np.array([[1.8, 0.0]], np.float32), np.ones((1, 3), np.float32), N, N, -40.0, -40.0,
        40.0, 40.0, 0.0, 0.0, 0.05, 60.0, 240, 6.283185307179586, 0.15, 0.35, 0.06, VMAX, 0.165,
        0.6, 0.12, 0.12, 0.8, 6, 1, 0.2, 4, 1, 2.2, -1.4, 8.0, 1, 0.5, 0.15, 4.0)
    pycvc.nav_sim_world_set_material(sim, risk, hard, 1, 0.5, 1.0, 1.25, 12.0, 1.0, 1, 16, 12, 1.0,
                                     0.05, 0.45, 0.5)
    return sim


def through_water(governor):
    """The agent's mean speed in the water (m/s), its ticks there, and whether it arrived."""
    sim = make_sim()
    if governor:
        pycvc.nav_sim_world_set_risk_governor(sim, 1, 0.0, 0.3, 0.5, 3)
    speeds = []
    for _ in range(800):
        pycvc.nav_sim_world_step(sim)
        pos, _h, speed, _m, reached = pycvc.nav_sim_world_snapshot(sim)
        c = int(round(float(pos[0][0]) + 40.0))  # snapshot() reads world metres
        if 0 <= c < N and hard[N // 2, c]:
            speeds.append(abs(float(speed[0])))
        if reached[0]:
            return float(np.mean(speeds)), len(speeds), True
    return float(np.mean(speeds)) if speeds else 0.0, len(speeds), False


def test_the_governor_slows_what_the_forces_do_not():
    off_v, off_n, off_arrived = through_water(False)
    on_v, on_n, on_arrived = through_water(True)
    top = VMAX / 0.05
    print("  through the water at %.1f m/s (%d ticks) without the governor, %.1f m/s (%d ticks) "
          "with it; top speed %.1f m/s" % (off_v, off_n, on_v, on_n, top))
    check(off_arrived and on_arrived, "both cross the water to the goal")
    check(off_v > 0.8 * top, "the forces alone do not slow it (%.1f m/s)" % off_v)
    check(on_v < 0.3 * top, "the governor does (%.1f m/s)" % on_v)
    check(on_n > 2 * off_n, "and it spends longer in the water")


def test_the_switch():
    sim = make_sim()
    check(pycvc.nav_sim_world_risk_governor(sim) is False, "off by default")
    pycvc.nav_sim_world_set_risk_governor(sim, 1)
    check(pycvc.nav_sim_world_risk_governor(sim) is True, "on")
    pycvc.nav_sim_world_set_risk_governor(sim, 0)
    check(pycvc.nav_sim_world_risk_governor(sim) is False, "off again")
    raises(lambda: pycvc.nav_sim_world_set_risk_governor(sim, 1, -0.1), "a negative vmin")
    raises(lambda: pycvc.nav_sim_world_set_risk_governor(sim, 1, 0.0, 1.0), "an onset of 1")
    raises(lambda: pycvc.nav_sim_world_set_risk_governor(sim, 1, 0.0, 0.3, -1.0),
           "a negative look-ahead")
    raises(lambda: pycvc.nav_sim_world_set_risk_governor(sim, 1, 0.0, 0.3, 0.5, 0), "no probes")


def main():
    for t in (test_the_governor_slows_what_the_forces_do_not, test_the_switch):
        t()
        print("ok  " + t.__name__)
    print("test_pycvc_nav_governor: %d checks passed" % checks)


if __name__ == "__main__":
    main()
