#!/usr/bin/env python3
"""pycvc.mujoco_robot: a MuJoCo robot (the Unitree Go2 from cvcpkg's mujoco-menagerie) on a
pycvc.height_field, from Python (docs/MUJOCO.md), as the C-PHAST lab's gym drives it.

Checks:
  1. without MuJoCo in the build, mujoco_available() is False and the constructor raises;
  2. the Go2 loads on a flat height field: its sizes, actuators, force ranges and servo targets;
     qpos, qvel and the commands come back as read-only float64 numpy copies;
  3. it stands under its servos (only the feet touch, and they carry its weight), and a 60 N push
     on its base moves it measurably and it recovers;
  4. the energy books close: MuJoCo's energy change matches the work done to 2e-5 of the gross;
  5. the gym's loop: reset() then the same commands replays an episode to the bit, and so does a
     restored snapshot;
  6. torque mode saturates at the motors' ranges; run_mujoco_trial gives the C++ trial's numbers
     as a dict, and the Go2 recovers on a 10 degree slope;
  7. step() releases the GIL: another Python thread runs while the robot steps; a wind
     (set_wind) pushes it along the wind, and still air not at all;
  8. bad input raises: an unknown mode, the wrong number of commands, energy on a robot built
     without it.
CVC_MUJOCO_MENAGERIE_DIR names the Menagerie (<prefix>/share/mujoco-menagerie; CMake sets it). Script
style, like the other pycvc tests; self-skips (exit 0) when the running pycvc predates the binding
or no Menagerie is given. On Windows, CVC_DLL_DIRS (os.pathsep-separated) names the directories
pycvc's DLLs load from.
"""

import math
import os
import sys
import threading

if sys.platform == "win32":
    for _d in os.environ.get("CVC_DLL_DIRS", "").split(os.pathsep):
        if _d and os.path.isdir(_d):
            os.add_dll_directory(_d)

import numpy as np
import pycvc

if not hasattr(pycvc, "mujoco_robot"):
    print("pycvc lacks mujoco_robot — skipping")
    raise SystemExit(0)

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


def expect_raises(fn, what):
    global checks
    try:
        fn()
    except (RuntimeError, ValueError, TypeError):
        checks += 1
        return
    raise AssertionError(what + ": no error raised")


def plane(slope_deg=0.0, half=3.0, cell=0.05):
    """A plane rising along +x at slope_deg, centred on the origin."""
    n = int(round(2.0 * half / cell)) + 1
    x = -half + cell * np.arange(n)
    h = np.tile(np.tan(math.radians(slope_deg)) * x, (n, 1)).astype(np.float32)
    return pycvc.height_field(n, n, -half, -half, cell, cell, h)


# ── 1. availability ──
if not pycvc.mujoco_available():
    check(pycvc.mujoco_version() == "", "no version without MuJoCo")
    expect_raises(lambda: pycvc.mujoco_robot("scene.xml", terrain=plane()), "robot without MuJoCo")
    print(f"pycvc.mujoco_robot: built without MuJoCo, {checks} checks passed")
    raise SystemExit(0)
check(pycvc.mujoco_version().startswith("MuJoCo 4."), pycvc.mujoco_version())

menagerie = os.environ.get("CVC_MUJOCO_MENAGERIE_DIR", "")
scene = os.path.join(menagerie, "unitree_go2", "scene.xml")
if not menagerie or not os.path.isfile(scene):
    print("no Menagerie (CVC_MUJOCO_MENAGERIE_DIR) — skipping")
    raise SystemExit(0)

# ── 2. the Go2 on a flat height field ──
field = plane()
go2 = pycvc.mujoco_robot(scene, terrain=field)
check(go2.nq() == 19 and go2.nv() == 18 and go2.actuators() == 12, "the Go2's sizes")
check(go2.terrain_geom() >= 0 and go2.geom("floor") == -1, "the floor is the height field")
check(abs(go2.mass() - (6.921 + 4 * (0.678 + 1.152 + 0.241352))) < 1e-9, "mass")
check(go2.timestep() == 0.002, "the model's step")
names = [go2.actuator_name(i) for i in range(12)]
check(names[:3] == ["FL_hip", "FL_thigh", "FL_calf"] and names[-1] == "RR_calf", names)
for i in range(12):
    limit = 45.43 if i % 3 == 2 else 23.7
    check(go2.force_range(i) == (-limit, limit), f"actuator {i}'s force range")
    check(go2.mode(i) == "position", f"actuator {i} starts as a servo")
home = np.tile([0.0, 0.9, -1.8], 4)
q = go2.qpos()
check(isinstance(q, np.ndarray) and q.dtype == np.float64 and q.shape == (19,), "qpos is numpy")
check(not q.flags.writeable, "qpos is a read-only copy")
check(np.array_equal(go2.commands(), home), "servo targets are the keyframe's angles")
check(np.array_equal(q[[go2.actuator_qpos(i) for i in range(12)]], home), "joints start home")
check(go2.qvel().shape == (18,), "qvel")
check(go2.contacts() == [], "no contacts before a step")

# ── 3. stands, and recovers from a push ──
go2.step(500)
b = go2.base()
check(b["up_z"] > 0.999, f"upright: {b['up_z']}")
height = b["position"][2] - field.height(b["position"][0], b["position"][1])
check(0.22 < height < 0.32, f"standing height {height}")
feet = {go2.geom(n) for n in ("FL", "FR", "RL", "RR")}
up = 0.0
for c in go2.contacts():
    t = go2.terrain_geom()
    check(t in c["geom"], "only the terrain is touched")
    other = c["geom"][1] if c["geom"][0] == t else c["geom"][0]
    check(other in feet, f"only feet touch the terrain, not {go2.geom_name(other)}")
    up += c["force"][2] if c["geom"][0] == t else -c["force"][2]
check(abs(up - 9.81 * go2.mass()) < 0.01 * 9.81 * go2.mass(), f"the feet carry the weight: {up}")
check(go2.actuator_forces().shape == (12,), "actuator forces")

y0 = go2.base()["position"][1]
base = go2.body("base")
go2.apply_wrench(base, (0.0, 60.0, 0.0))
go2.step(50)
go2.clear_wrenches()
moved = 0.0
for _ in range(150):  # 3 s, read every 20 ms
    go2.step(10)
    moved = max(moved, abs(go2.base()["position"][1] - y0))
b = go2.base()
speed = math.sqrt(sum(v * v for v in b["velocity"]))
check(moved > 0.01, f"the push moved it: {moved:.4f} m")
check(b["up_z"] > 0.995 and speed < 0.05, f"it recovered: up_z {b['up_z']}, speed {speed}")
print(f"stand + 60 N push: moved up to {moved:.4f} m, ends at {speed:.2e} m/s")

# ── 4. the energy books ──
k = go2.books()
gross = sum(abs(k[n]) for n in ("actuators", "external", "contacts", "joint_friction",
                                "joint_limits", "joint_damping", "other"))
e = go2.energy()
check(abs(e["total"] - k["energy_now"]) < 1e-12, "books read MuJoCo's energy")
check(abs(k["energy_now"] - k["energy_start"] - k["work"] - k["residual"]) < 1e-12, "residual")
check(abs(k["residual"]) < 2e-5 * gross, f"books close: {k['residual']:.3e} of {gross:.3f} J")
check(k["external"] > 0.0 and k["contacts"] < 0.0 and k["joint_damping"] < 0.0, "signs")
print(f"books: residual {k['residual']:.3e} J of {gross:.3f} J gross")


# ── 5. the gym's loop: reset and replay ──
def episode(robot, steps=400):
    robot.reset()
    for s in range(steps):
        u = home + 0.25 * math.sin(2.0 * math.pi * 1.5 * robot.time()) * np.tile([0.0, 1.0, -2.0], 4)
        robot.command(u)
        if s == 100:
            robot.apply_wrench(base, (0.0, 40.0, 0.0))
        if s == 150:
            robot.clear_wrenches()
        robot.step()
    return np.concatenate([robot.qpos(), robot.qvel()])


first = episode(go2)
check(go2.time() > 0.79, "the episode ran")
go2.reset()
check(go2.time() == 0.0 and go2.contacts() == [] and go2.books()["work"] == 0.0, "reset")
check(np.array_equal(go2.commands(), home), "reset restores the servo targets")
second = episode(go2)
check(np.array_equal(first, second), "reset replays an episode to the bit")
other = pycvc.mujoco_robot(scene, terrain=field)
check(np.array_equal(first, episode(other)), "a second robot replays it too")

go2.reset()
go2.step(200)
snap = go2.save()
go2.step(300)
after = np.concatenate([go2.qpos(), go2.qvel()])
go2.restore(snap)
go2.step(300)
check(np.array_equal(after, np.concatenate([go2.qpos(), go2.qvel()])), "restore replays")

# ── 6. torque mode, and the trial ──
go2.reset()
go2.set_mode("torque")
check(all(go2.mode(i) == "torque" for i in range(12)) and not go2.commands().any(), "torque")
go2.command([100.0] * 12)
go2.step()
check(np.array_equal(go2.actuator_forces(), [go2.force_range(i)[1] for i in range(12)]),
      "motors saturate at their ranges")
go2.set_mode(1, "position")
go2.set_gains(1, 30.0, 1.0)
check(go2.mode(1) == "position" and go2.kp(1) == 30.0 and go2.kd(1) == 1.0, "one servo")

t = pycvc.run_mujoco_trial(scene, slope_deg=10.0, push=(0.0, 60.0, 0.0), duration=3.0)
check(t["recovered"] and not t["fell"] and t["steps"] == 1500, f"trial: {t}")
check(abs(t["books"]["residual"]) < 1e-3 and t["max_displacement"] > 0.01, "trial's numbers")
print(f"trial on 10 degrees: moved up to {t['max_displacement']:.4f} m, recovered")

# ── 7. step() without the GIL ──
spins = 0
spinning = True


def spin():
    global spins
    while spinning:
        spins += 1


th = threading.Thread(target=spin)
th.start()
go2.reset()
before = spins
go2.step(1000)  # about 0.3 s, which a thread blocked on the GIL would sit out
during = spins - before
spinning = False
th.join()
check(during > 1000, f"another thread ran while the robot stepped: {during} spins")

# ── 7b. the wind: the scene's air (a pycvc.wind_flow) pushes the robot along it ──
if hasattr(go2, "set_wind"):
    go2.reset()
    go2.set_wind(pycvc.wind_flow(speed_mps=15.0, dir_rad=math.pi / 2, gustiness=0.0))
    go2.step(10)
    f = go2.wind_force()
    check(f[1] > 1.0 and abs(f[0]) < 0.01 * f[1], f"a 15 m/s wind along +y pushes it: {f}")
    go2.set_wind(None)
    go2.step(1)
    check(go2.wind_force() == (0.0, 0.0, 0.0), "still air pushes nothing")
    print(f"15 m/s wind: {f[1]:.2f} N along it")

# ── 8. bad input ──
expect_raises(lambda: go2.set_mode("walk"), "an unknown mode")
expect_raises(lambda: go2.command([0.0] * 11), "eleven commands")
expect_raises(lambda: go2.command(12, 0.0), "no actuator 12")
expect_raises(lambda: go2.apply_wrench(0, (1.0, 0.0, 0.0)), "pushing the world")
expect_raises(lambda: pycvc.mujoco_robot(scene, terrain=field, floor="none"), "no such floor")
quiet = pycvc.mujoco_robot(scene, terrain=field, energy=False)
expect_raises(quiet.energy, "energy when built without it")

print(f"pycvc.mujoco_robot: {checks} checks passed")
