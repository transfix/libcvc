"""pycvc.WorldEntities: a Python host places a world's `pose` entities, as an outside simulation.

Headless (pycvc.AriApp, no GL): a document whose world: block offers a Humvee with the `pose`
controller (no plant integrates it; its chassis is a kinematic box at each pose). From Python:
  * spawn() makes one, not placed until its first pose; list() shows it as posed;
  * set_pose() puts it exactly where it says (bit for bit), standing on the ground with no z,
    its speed and steer listed; pose() is what was given;
  * timed poses are drawn between the last two, a span behind;
  * set_poses() places many at once from a numpy array (rows of x y [z heading ...]);
  * set_poses() from another thread, while frames run, is safe, and the last one holds;
  * the refusals: not a pose entity, a number that is not finite, rows of the wrong size.
"""

import math
import os
import sys
import tempfile
import threading

import pycvc

if not hasattr(pycvc, "WorldEntities") or not hasattr(pycvc.WorldEntities, "set_pose"):
    print("SKIP test_pycvc_world_poses: this pycvc has no WorldEntities.set_pose")
    sys.exit(0)
if not pycvc.AriApp.have_yaml():
    print("SKIP test_pycvc_world_poses: libcvc built without yaml-cpp")
    sys.exit(0)
if "jolt" not in pycvc.physics_backends() or not pycvc.physics_backend_info("jolt")["available"]:
    print("SKIP test_pycvc_world_poses: no Jolt in this build")
    sys.exit(0)

import numpy as np

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


def raises(fn, what):
    try:
        fn()
    except (RuntimeError, ValueError, TypeError):
        check(True, what)
        return
    raise AssertionError(f"no error: {what}")


DOC = """meta: { name: world poses test, min_libcvc: 4.0.1 }
requires: [scene.physics]
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee, controllers: [pose, nav@1] }
scene:
  clock: { paused: true }
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [160, 160, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
"""

tmp = tempfile.mkdtemp(prefix="pycvc_world_poses_")
path = os.path.join(tmp, "poses.ari")
with open(path, "w", encoding="utf-8") as f:
    f.write(DOC)


def frames(rt, n):
    """n frames of world time (1/60 s each): the paused clock's quanta (1/120 s), then a drain."""
    rt.step_world(2 * n)
    rt.step(1.0 / 60.0)


def by_id(ents, i):
    return next(e for e in ents.list() if e["id"] == i)


app = pycvc.make_app()
rt = pycvc.AriApp(app, "w")
warnings = rt.load(path)
check(not [w for w in warnings if "world:" in w], f"the document loads: {warnings}")
rt.step(1.0 / 60.0)
ents = rt.entities()
check(ents.available(), "the headless runtime runs a world with vehicles")

# One with no pose yet: listed, not placed.
a = ents.spawn("humvee", name="lead")
check(a == 1, f"spawned ({ents.why()})")
frames(rt, 5)
e = by_id(ents, a)
check(e["controller"] == "pose" and not e["placed"] and not e["posed"], f"waits: {e}")
check(e["pose"] is None, "no pose yet")
check(ents.pose(a) is None, "pose(): none given")

# Untimed: exactly where it says at the next step, on the ground, its speed and steer as given.
check(ents.set_pose(a, 12.25, -7.5, heading=0.75, speed=9.5, steer=0.125), "set_pose")
frames(rt, 1)
e = by_id(ents, a)
check(e["placed"] and e["posed"], f"placed: {e}")
check(e["x"] == 12.25 and e["y"] == -7.5 and e["yaw"] == 0.75, f"exactly there: {e}")
check(e["speed"] == 9.5 and e["steer"] == 0.125, f"its speed and steer: {e}")
check(0.5 < e["z"] < 2.0, f"standing on the ground: z {e['z']}")
check(e["pose"]["speed"] == 9.5 and e["pose"]["t"] is None, f"its pose: {e['pose']}")
check(pycvc.state_get(app, "w.entities.1.command.by") == "pose", "placed from outside")
given = ents.pose(a)
check(given["x"] == 12.25 and given["z"] is None and not given["rear_axle"], f"pose(): {given}")
# Given z, pitch and roll.
check(ents.set_pose(a, 0.0, 0.0, z=3.0, pitch=0.1, roll=-0.05), "with z, pitch, roll")
frames(rt, 1)
e = by_id(ents, a)
check(e["z"] == 3.0 and abs(e["pose"]["pitch"] - 0.1) < 1e-15, f"z and pitch: {e['pose']}")

# Timed: drawn between the last two, reaching the second a span (0.25 s) after it came.
check(ents.set_pose(a, 0.0, 10.0, t=1.0), "a timed pose")
frames(rt, 1)
check(by_id(ents, a)["y"] == 10.0, "the first timed one: at once")
check(ents.set_pose(a, 5.0, 10.0, t=1.25), "the next, 0.25 s on")
frames(rt, 6)  # 0.1 s
x = by_id(ents, a)["x"]
check(1.5 < x < 2.5, f"two fifths of the way at 0.1 s: {x}")
frames(rt, 12)  # 0.3 s in all
check(by_id(ents, a)["x"] == 5.0, "there at the span's end, and held")

# Many at once, from numpy: rows of x y z heading.
ids = [a] + [ents.spawn("humvee", name=f"car {i}") for i in range(1, 6)]
check(all(ids), f"five more ({ents.why()})")
poses = np.zeros((len(ids), 4))
poses[:, 0] = np.arange(len(ids)) * 6.0 - 15.0
poses[:, 1] = 30.0
poses[:, 2] = np.nan  # on the ground
poses[:, 3] = 1.5
check(ents.set_poses(ids, poses) == len(ids), "all of them taken")
frames(rt, 1)
for i, eid in enumerate(ids):
    e = by_id(ents, eid)
    check(e["x"] == i * 6.0 - 15.0 and e["y"] == 30.0 and e["yaw"] == 1.5, f"{eid}: {e}")
# Rear axles: the centre ahead of each along its heading.
check(ents.set_poses([ids[1]], [[0.0, 40.0, float("nan"), 0.0]], rear_axle=True) == 1, "rear")
frames(rt, 1)
check(by_id(ents, ids[1])["x"] > 1.0, f"its centre ahead: {by_id(ents, ids[1])['x']}")

# From another thread while frames run: the last one holds.
done = threading.Event()


def writer():
    for k in range(301):
        p = np.zeros((len(ids), 2))
        p[:, 0] = k * 0.1
        p[:, 1] = np.arange(len(ids)) * 5.0
        ents.set_poses(np.array(ids, dtype=np.uint64), p)
    done.set()


t = threading.Thread(target=writer)
t.start()
while not done.is_set():
    frames(rt, 1)
t.join()
frames(rt, 2)
for i, eid in enumerate(ids):
    e = by_id(ents, eid)
    check(e["x"] == 30.0 and e["y"] == i * 5.0, f"the last batch holds: {eid} {e['x']}, {e['y']}")

# The refusals.
n = ents.spawn("humvee", name="driven", controller="nav@1")
check(n and not ents.set_pose(n, 0.0, 0.0), "a nav@1 entity takes no pose")
check(not ents.set_pose(999, 0.0, 0.0), "not an entity")
check(ents.set_poses([n, 999], [[0.0, 0.0], [1.0, 1.0]]) == 0, "none of them taken")
raises(lambda: ents.set_pose(a, math.inf, 0.0), "x not finite")
raises(lambda: ents.set_pose(a, 0.0, 0.0, z=math.inf), "z not finite")
raises(lambda: ents.set_poses([a], [[1.0]]), "a row of one number")
raises(lambda: ents.set_poses([a, ids[1]], [[1.0, 2.0]]), "a row short")
raises(lambda: ents.set_pose_jump(0.0), "a jump of 0 m")
check(ents.set_pose_jump(50.0), "set_pose_jump")

print(f"test_pycvc_world_poses: OK ({checks} checks)")
