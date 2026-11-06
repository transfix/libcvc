#!/usr/bin/env python3
"""pycvc: the sim_world's fog-of-war surface from Python.

A world that changes under its agents, and what they know of it: ``nav_sim_world_truth`` and
``nav_sim_world_set_truth`` (a wall raised and taken down mid-run), the fog tiers
``nav_sim_world_ever_seen`` / ``nav_sim_world_last_visible``, and ``nav_sim_world_add_obstacle``
with its decaying marks in ``nav_sim_world_dynamic_stamps``. Script-style, like the other pycvc
tests; self-skips (exit 0) when the running pycvc predates the bindings. On Windows, CVC_DLL_DIRS
(os.pathsep-separated) names the directories pycvc's DLLs load from.
"""

import os
import sys

if sys.platform == "win32":
    for _d in os.environ.get("CVC_DLL_DIRS", "").split(os.pathsep):
        if _d and os.path.isdir(_d):
            os.add_dll_directory(_d)

import numpy as np

import pycvc

if not hasattr(pycvc, "nav_sim_world_set_truth"):
    print("pycvc lacks nav_sim_world_set_truth — skipping")
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
    except (RuntimeError, ValueError, IndexError):
        global checks
        checks += 1
        return
    raise AssertionError(what + ": no error raised")


# 81 x 81 one-metre cells, scale 0.05, one agent at the centre told to stay there, sensing every
# other tick out to 30 m (test_pycvc_physics.py's world, with the sensor on).
N = 81
open_field = np.zeros((N, N), np.uint8)
here = np.array([[0.0, 0.0]], np.float32)
sim = pycvc.nav_sim_world_create(
    open_field, open_field, "", here, here, np.ones((1, 3), np.float32), N, N, -40.0, -40.0,
    40.0, 40.0, 0.0, 0.0, 0.05, 30.0, 240, 6.283185307179586, 0.15, 0.35, 0.06, 0.4, 0.165, 0.6,
    0.12, 0.12, 0.8, 6, 1, 0.2, 2, 0, 2.2, -1.4, 8.0, 1, 0.5, 0.15, 4.0)
for _ in range(2):
    pycvc.nav_sim_world_step(sim)

seen = pycvc.nav_sim_world_ever_seen(sim, 0)
vis = pycvc.nav_sim_world_last_visible(sim, 0)
check(seen.shape == (N, N) and seen.dtype == np.uint8, "ever_seen is (rows, cols) uint8")
check(seen[40, 40] == 1 and seen[40, 60] == 1 and seen[40, 75] == 0,
      "the agent has seen around it out to its 30 m range, not past it")
check(np.array_equal(vis.astype(bool) & ~seen.astype(bool), np.zeros_like(seen, bool)),
      "what is in view is also remembered")
check(not pycvc.nav_sim_world_truth(sim).any(), "an open field")

# A wall raised 5 m east: truth has it at once, the belief from the next sense tick.
wall = open_field.copy()
wall[34:47, 45:47] = 1
pycvc.nav_sim_world_set_truth(sim, wall.astype(bool))
check(np.array_equal(pycvc.nav_sim_world_truth(sim), wall), "truth reads the new wall back")
check(not (pycvc.nav_sim_world_belief_occ(sim, 0) & wall).any(), "not believed before sensing")
for _ in range(2):
    pycvc.nav_sim_world_step(sim)
believed = int((pycvc.nav_sim_world_belief_occ(sim, 0) & wall).sum())
check(believed > 4, "its near face is believed after a sense tick (%d cells)" % believed)

# Taken down, the belief clears where the sensor sees free space.
pycvc.nav_sim_world_set_truth(sim, open_field)
for _ in range(24):
    pycvc.nav_sim_world_step(sim)
check(int((pycvc.nav_sim_world_belief_occ(sim, 0) & wall).sum()) < believed,
      "the wall's cells clear once seen empty")

# A decaying mark: stamped now, believed at the next sense tick, gone after ttl_s (4 s).
stamps = pycvc.nav_sim_world_dynamic_stamps(sim, 0)
check(stamps.dtype == np.float64 and np.isneginf(stamps).all(), "no marks yet")
pycvc.nav_sim_world_add_obstacle(sim, 30, 32, 30, 32)
stamps = pycvc.nav_sim_world_dynamic_stamps(sim, 0)
t_now = pycvc.nav_sim_world_tick(sim) * 0.06
check(np.isclose(stamps[31, 31], t_now) and np.isneginf(stamps[10, 10]),
      "the blob is stamped with the sim time, nothing else is")
for _ in range(2):
    pycvc.nav_sim_world_step(sim)
check(pycvc.nav_sim_world_belief_occ(sim, 0)[31, 31] == 1, "the mark is believed")
for _ in range(80):  # 4.8 s
    pycvc.nav_sim_world_step(sim)
check(pycvc.nav_sim_world_belief_occ(sim, 0)[31, 31] == 0, "and expires")

raises(lambda: pycvc.nav_sim_world_set_truth(sim, np.zeros((3, 3), np.uint8)), "a wrong shape")
raises(lambda: pycvc.nav_sim_world_ever_seen(sim, 1), "a plane the world lacks")
raises(lambda: pycvc.nav_sim_world_dynamic_stamps(sim, -1), "a negative plane")
print("test_pycvc_nav_fog: %d checks passed" % checks)
